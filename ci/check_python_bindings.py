#!/usr/bin/env python3
"""Check newly introduced public C++ declarations for Python binding evidence.

This is a source-level precheck, not a substitute for building/testing pybind11.
See bindings/python/README.md for scope, limitations, and exceptions.
"""

import argparse
from collections import defaultdict
from dataclasses import dataclass
import json
from pathlib import Path
import re
import subprocess
import sys

from tree_sitter import Language, Parser
import tree_sitter_cpp


PARSER = Parser(Language(tree_sitter_cpp.language()))
ROOT = Path(__file__).resolve().parents[1]
EXCEPTIONS = "ci/python_binding_exceptions.json"


def walk(node):
    yield node
    for child in node.named_children:
        yield from walk(child)


def parse(source, keep_conditionals=False):
    # Headers need conditional structure to preserve branch-specific access.
    # Binding expression chains are inspected across all branches together.
    # Preserve positions so diagnostics still point to the original source.
    directive = r"^[ \t]*#"
    if keep_conditionals:
        directive += r"(?!\s*(?:if|ifdef|ifndef|elif|else|endif)\b)"
    pattern = (
        r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\''
        r'|//[^\n]*|/\*[\s\S]*?\*/|\[\[[\s\S]*?\]\]'
        r'|' + directive + r'(?:[^\n]*\\\n)*[^\n]*'
        r'|\bDEPTHAI_(?:BEGIN|END)_SUPPRESS_DEPRECATION_WARNING\b'
    )
    source = re.sub(pattern, lambda m: m[0] if m[0][0] in "\"'" else re.sub(r"[^\n]", " ", m[0]), source, flags=re.M)
    return PARSER.parse(source.encode()).root_node


def function_signature(function):
    parameters = function.child_by_field_name("parameters")
    types = []
    for param in parameters.named_children:
        # Parameter names/defaults are not overload identities.
        end = param.child_by_field_name("default_value")
        value = param.text.decode() if end is None else param.text[:end.start_byte - param.start_byte].decode().rstrip(" =")
        declarator = param.child_by_field_name("declarator")
        while declarator is not None:
            inner = declarator.child_by_field_name("declarator")
            if inner is None and declarator.type in ("reference_declarator", "parenthesized_declarator"):
                inner = declarator.named_children[0]
            if inner is None:
                break
            declarator = inner
        if declarator is not None and declarator.type in ("identifier", "field_identifier"):
            start = declarator.start_byte - param.start_byte
            value = value[:start] + value[start + len(declarator.text):]
        types.append(re.sub(r"\s+", "", value))
    qualifiers = "".join(c.text.decode() for c in function.named_children if c.type in ("type_qualifier", "ref_qualifier"))
    if types == ["void"]:
        types = []
    return "(" + ",".join(types) + ")" + re.sub(r"\s+", "", qualifiers)


@dataclass(frozen=True)
class Symbol:
    name: str
    signature: str
    path: str
    line: int

    @property
    def key(self):
        return self.name + self.signature


def public_api(source, path):
    symbols = {}
    errors = set()

    def visit(node, scope=(), access=True):
        if node.type == "ERROR":
            errors.add((path, re.sub(r"\s+", "", node.text.decode())))
            return
        if node.type == "access_specifier":
            return node.text == b"public"
        if node.type in ("preproc_if", "preproc_ifdef", "preproc_elif", "preproc_else"):
            # Each alternative starts with the incoming access, not its sibling's
            # final access. After #endif, retain visibility in any possible branch.
            alternative = node.child_by_field_name("alternative")
            branch_access = access
            for child in node.named_children:
                if child == alternative:
                    continue
                new_access = visit(child, scope, branch_access)
                if new_access is not None:
                    branch_access = new_access
            if node.type == "preproc_else":
                return branch_access
            other_access = visit(alternative, scope, access) if alternative else access
            return branch_access or other_access
        if not access and node.type != "field_declaration_list":
            return
        if node.type == "namespace_definition":
            name_node = node.child_by_field_name("name")
            name = name_node.text.decode() if name_node else ""
            if name and not {"detail", "internal", "impl"}.intersection(name.split("::")):
                visit(node.child_by_field_name("body"), (*scope, name))
            return
        if node.type in ("class_specifier", "struct_specifier", "enum_specifier"):
            name_node = node.child_by_field_name("name")
            name = name_node.text.decode() if name_node else ""
            body = node.child_by_field_name("body")
            if body is None:
                return
            if not name:
                if node.type == "enum_specifier":
                    visit(body, scope)
                return
            qualified = "::".join((*scope, name))
            symbol = Symbol(qualified, "", path, node.start_point.row + 1)
            symbols[symbol.key] = symbol
            visit(body, (*scope, name), node.type != "class_specifier")
            return
        if node.type == "enumerator":
            name = node.child_by_field_name("name").text.decode()
            symbol = Symbol("::".join((*scope, name)), "", path, node.start_point.row + 1)
            symbols[symbol.key] = symbol
            return
        if node.type in ("field_declaration", "declaration", "function_definition"):
            for decl in node.children_by_field_name("declarator"):
                current = decl
                function = None
                while current is not None:
                    if current.type == "function_declarator":
                        function = current
                    next_decl = current.child_by_field_name("declarator")
                    if next_decl is None and current.type in ("reference_declarator", "parenthesized_declarator"):
                        next_decl = current.named_children[0]
                    if next_decl is None:
                        break
                    current = next_decl
                name = current.text.decode()
                if not re.fullmatch(r"[A-Za-z_]\w*", name):
                    continue  # Operators and destructors need manual review.
                if function is not None:
                    if "delete_method_clause" in {c.type for c in node.named_children}:
                        continue
                    signature = function_signature(function)
                else:
                    signature = ""
                symbol = Symbol("::".join((*scope, name)), signature, path, node.start_point.row + 1)
                symbols[symbol.key] = symbol
            # Nested types occur inside field declarations; never inspect bodies,
            # initializers, parameters or local variables as public declarations.
            for child in node.named_children:
                if child.type in ("class_specifier", "struct_specifier", "enum_specifier", "ERROR"):
                    visit(child, scope, access)
            return
        if node.type in ("translation_unit", "declaration_list", "field_declaration_list", "enumerator_list", "template_declaration", "linkage_specification"):
            for child in node.named_children:
                new_access = visit(child, scope, access)
                if new_access is not None:
                    access = new_access
            return access

    visit(parse(source, keep_conditionals=True))
    return {key: value for key, value in symbols.items() if key.startswith("dai::")}, errors


def binding_evidence(source):
    """Map C++ names to (optional signature, callable) registrations."""
    root = parse(source)
    evidence = defaultdict(set)
    global_usings = [n for n in root.named_children if n.type in ("using_declaration", "alias_declaration")]
    for function in (n for n in walk(root) if n.type == "function_definition"):
        nodes = list(walk(function))
        namespaces = {""}
        aliases = {}
        objects = {}
        for node in global_usings + nodes:
            if node.type == "using_declaration":
                match = re.fullmatch(r"using\s+namespace\s+([\w:]+)\s*;", node.text.decode())
                if match:
                    namespaces.add(match[1] + "::")
            elif node.type == "alias_declaration":
                aliases[node.child_by_field_name("name").text.decode()] = re.sub(r"\s+", "", node.child_by_field_name("type").text.decode())

        def qualify(name):
            first, sep, rest = name.partition("::")
            if first in aliases:
                name = aliases[first] + (sep + rest if sep else "")
            if name.startswith("dai::"):
                return {name}
            return {prefix + name for prefix in namespaces}

        def owner(node):
            if node is None:
                return set()
            if node.type in ("identifier", "field_identifier"):
                return objects.get(node.text.decode(), set())
            if node.type == "call_expression":
                callee = node.child_by_field_name("function")
                if callee.type == "field_expression":
                    return owner(callee.child_by_field_name("argument"))
                match = re.match(r"(?:py|pybind11)::(?:class_|enum_)<([^,>]+)", re.sub(r"\s+", "", callee.text.decode()))
                if match:
                    return qualify(match[1])
                if re.fullmatch(r"ADD_(?:BETA_)?NODE(?:_\w+)?", callee.text.decode()):
                    return qualify(node.child_by_field_name("arguments").named_children[0].text.decode())
            return set()

        for node in nodes:
            if node.type == "declaration":
                type_node = node.child_by_field_name("type")
                type_name = type_node.text.decode() if type_node else ""
                match = re.match(r"(?:py|pybind11)::(?:class_|enum_)<([^,>]+)", re.sub(r"\s+", "", type_name))
                for decl in node.children_by_field_name("declarator"):
                    names = qualify(match[1]) if match else owner(decl.child_by_field_name("value"))
                    if names:
                        objects[decl.child_by_field_name("declarator").text.decode()] = names
                        for name in names:
                            evidence[name].add((None, "type"))
            if node.type != "call_expression":
                continue
            callee = node.child_by_field_name("function")
            owners = owner(node)
            if callee.type != "field_expression":
                for name in owners:
                    evidence[name].add((None, "type"))
                continue
            method = callee.child_by_field_name("field").text.decode()
            if not (method.startswith("def") or method == "value"):
                continue
            args = node.child_by_field_name("arguments")
            if not args.named_children:
                continue
            first = args.named_children[0]
            constructor = re.match(r"(?:py|pybind11)::init(?:_alias)?[<(]", re.sub(r"\s+", "", first.text.decode()))
            # Only inspect callables/values, never Python names, DOCs or default arguments.
            if constructor:
                callbacks = args.named_children[:1]
            else:
                end = 3 if method in ("def_property", "def_property_static") else 2
                callbacks = args.named_children[1:end]
            for callback in callbacks:
                targets = set()
                signature = None
                if constructor:
                    targets.update(name + "::" + name.rsplit("::", 1)[-1] for name in owners)
                elif method == "value":
                    if callback.type in ("identifier", "qualified_identifier"):
                        value = re.sub(r"\s+", "", callback.text.decode())
                        member = value.rsplit("::", 1)[-1]
                        references = qualify(value)
                        for enum in owners:
                            # Unscoped enumerators may also be named through the
                            # enclosing namespace/class, or without qualification.
                            enclosing = enum.rpartition("::")[0]
                            unscoped = enclosing + "::" + member if enclosing else member
                            if callback.type == "identifier" or {enum + "::" + member, unscoped} & references:
                                targets.add(enum + "::" + member)
                elif callback.type == "lambda_expression":
                    declarator = callback.child_by_field_name("declarator")
                    parameters = declarator.child_by_field_name("parameters") if declarator else None
                    if parameters is None or not parameters.named_children:
                        continue
                    instance = parameters.named_children[0]
                    type_node = instance.child_by_field_name("type")
                    instance_decl = instance.child_by_field_name("declarator")
                    if type_node is None or instance_decl is None:
                        continue
                    type_name = re.sub(r"\s+", "", type_node.text.decode())
                    type_name = re.sub(r"^std::(?:shared_ptr|unique_ptr)<(.+)>$", r"\1", type_name)
                    instance_types = owners & qualify(type_name)
                    names = [n.text for n in walk(instance_decl) if n.type == "identifier"]
                    if not instance_types or len(names) != 1:
                        continue
                    for ref in walk(callback.child_by_field_name("body")):
                        if ref.type != "field_expression":
                            continue
                        receiver = ref.child_by_field_name("argument")
                        if receiver.type != "identifier" or receiver.text != names[0]:
                            continue
                        # A nested lambda may shadow the instance parameter.
                        scope = ref.parent
                        while scope is not None and scope.type != "lambda_expression":
                            scope = scope.parent
                        if scope == callback:
                            member = ref.child_by_field_name("field").text.decode()
                            targets.update(name + "::" + member for name in instance_types)
                else:
                    for ref in walk(callback):
                        if ref.type in ("identifier", "qualified_identifier") and ref.parent.type == "pointer_expression":
                            targets.update(qualify(re.sub(r"\s+", "", ref.text.decode())))

                # Explicit overload casts and constructors give usable C++ parameter types.
                callee = callback.child_by_field_name("function")
                if callee is not None and re.match(
                    r"(?:py|pybind11)::(?:overload_cast|init|init_alias)<", re.sub(r"\s+", "", callee.text.decode())
                ):
                    template = callee.child_by_field_name("name")
                    types = template.child_by_field_name("arguments").named_children
                    signature = "(" + ",".join(re.sub(r"\s+", "", t.text.decode()) for t in types) + ")"
                    call_args = callback.child_by_field_name("arguments").named_children
                    if len(call_args) > 1 and re.fullmatch(
                        r"(?:py|pybind11)::const_", re.sub(r"\s+", "", call_args[1].text.decode())
                    ):
                        signature += "const"
                elif callee is not None and callee.type == "template_function" and callee.child_by_field_name("name").text == b"static_cast":
                    # Tree-sitter misparses member-pointer types inside template
                    # arguments. Reparse the exact type as a named declaration.
                    cast_type = callee.child_by_field_name("arguments").text[1:-1].decode()
                    declaration, replaced = re.subn(r"\(\s*(?:[^()]*?::\s*)?\*\s*\)", " _binding_signature", cast_type, count=1)
                    signature = ""  # An unresolved explicit cast cannot cover an arbitrary overload.
                    if replaced:
                        tree = parse("struct Binding { " + declaration + "; };")
                        if not tree.has_error:
                            for function in walk(tree):
                                if function.type == "function_declarator" and function.child_by_field_name("declarator").text == b"_binding_signature":
                                    signature = function_signature(function)
                                    break
                # Aliases/docstrings for one callable must not count as extra overloads.
                fingerprint = re.sub(r"\s+", "", callback.text.decode())
                for name in targets:
                    evidence[name].add((signature, fingerprint))
    return evidence


def git(*args):
    return subprocess.check_output(["git", "-C", str(ROOT), *args]).decode()


def check(base):
    base = git("rev-parse", "--verify", base + "^{commit}").strip()
    paths = set(git("diff", "--no-renames", "--name-only", "-z", base, "--", "include/depthai").split("\0"))
    paths.update(git("ls-files", "--others", "--exclude-standard", "-z", "--", "include/depthai").split("\0"))
    old_paths = set(git("ls-tree", "-r", "--name-only", base).splitlines())
    before, after = {}, {}
    old_errors, new_errors = set(), set()
    for path in sorted(paths):
        if Path(path).suffix not in (".hpp", ".h") or {"internal", "test"}.intersection(Path(path).parts):
            continue
        if path in old_paths:
            api, errors = public_api(git("show", base + ":" + path), path)
            before.update(api)
            old_errors.update(errors)
        if (ROOT / path).is_file():
            api, errors = public_api((ROOT / path).read_text(), path)
            after.update(api)
            new_errors.update(errors)
    exceptions = json.loads((ROOT / EXCEPTIONS).read_text())
    if not isinstance(exceptions, dict) or any(not isinstance(reason, str) or not reason.strip() for reason in exceptions.values()):
        raise ValueError(f"{EXCEPTIONS} must map exact symbol keys to nonempty reasons")
    added = [symbol for key, symbol in after.items() if key not in before and key not in exceptions]
    problems = [f"{path}: cannot inspect new C++ syntax: {error[:100]}" for path, error in sorted(new_errors - old_errors)]
    if added:
        bindings = []
        for revision in (base, None):
            evidence = defaultdict(set)
            cmake_path = "bindings/python/CMakeLists.txt"
            cmake = git("show", revision + ":" + cmake_path) if revision else (ROOT / cmake_path).read_text()
            sources = set(re.findall(r"\bsrc/[\w/]+\.cpp\b", re.sub(r"#[^\n]*", "", cmake)))
            for source in sorted(sources):
                path = "bindings/python/" + source
                content = git("show", revision + ":" + path) if revision else (ROOT / path).read_text()
                for name, registrations in binding_evidence(content).items():
                    evidence[name].update(registrations)
            bindings.append(evidence)
        old_names = {symbol.name for symbol in before.values()}
        groups = defaultdict(list)
        for symbol in added:
            groups[symbol.name].append(symbol)
        for name, symbols in sorted(groups.items()):
            registrations = bindings[1].get(name, set())
            reason = "missing Python binding"
            if registrations:
                if not symbols[0].signature:
                    continue
                candidates = registrations - bindings[0].get(name, set()) if name in old_names else registrations
                signatures = {signature for signature, _ in candidates if signature is not None}
                symbols = [symbol for symbol in symbols if symbol.signature not in signatures]
                if len(symbols) <= sum(signature is None for signature, _ in candidates):
                    continue
                reason = "new overload/signature needs a distinct applicable binding or an explicit exception"
            for symbol in sorted(symbols, key=lambda s: (s.path, s.line, s.key)):
                problems.append(f"{symbol.path}:{symbol.line}: {symbol.key}: {reason}")
    for problem in problems:
        print(problem, file=sys.stderr)
    if problems:
        print(f"Add bindings, or document intentional C++-only APIs by exact key in {EXCEPTIONS}.", file=sys.stderr)
    else:
        print(f"Python binding precheck passed ({len(added)} new public declarations checked).")
    return bool(problems)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", default="HEAD^", help="Comparison commit (use the PR merge base; defaults to HEAD^)")
    args = parser.parse_args()
    try:
        return check(args.base)
    except (ValueError, OSError, subprocess.CalledProcessError) as error:
        print(f"Python binding precheck failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
