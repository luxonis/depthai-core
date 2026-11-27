#!/usr/bin/env python3
import argparse
from dataclasses import dataclass
import io
from pathlib import Path
import re
import subprocess
import sys
import tokenize

from tree_sitter import Language, Parser
import tree_sitter_cpp

ROOT = Path(__file__).resolve().parents[1]
PARSER = Parser(Language(tree_sitter_cpp.language()))

CPP_SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx", ".inl", ".ipp"}
PYTHON_SUFFIXES = {".py"}
SKIPPED_FOLDERS = {"3rdparty", "external", "examples", "test", "tests"}
SKIPPED_PREFIXES = (
    "device/rvc4/src/vendor/",
    "generated/",
    "shared/",
    "src/bspatch/",
    "include/depthai/utility/span.hpp",
    "include/depthai/utility/spimpl.h",
)
UNCHECKED_KINDS = {"todo", "trailing"}
ALLOWED_KINDS = {"api doc", "directive", "legal", "namespace end"} | UNCHECKED_KINDS
MESSAGES = {
    "comment": "comment not allowed",
    "doc": "doc comment not allowed here: only public declarations in include/**/*.hpp can have one",
    "todo": "TODO comment not allowed",
    "trailing": "trailing comment not allowed",
}

DECLARATIONS = {
    "alias_declaration",
    "class_specifier",
    "concept_definition",
    "declaration",
    "enum_specifier",
    "enumerator",
    "field_declaration",
    "friend_declaration",
    "function_definition",
    "namespace_definition",
    "preproc_def",
    "preproc_function_def",
    "struct_specifier",
    "template_declaration",
    "type_definition",
    "union_specifier",
    "using_declaration",
}
DECLARATION_SCOPES = {
    "class_specifier",
    "declaration",
    "declaration_list",
    "enum_specifier",
    "enumerator_list",
    "field_declaration",
    "field_declaration_list",
    "linkage_specification",
    "namespace_definition",
    "preproc_elif",
    "preproc_elifdef",
    "preproc_else",
    "preproc_if",
    "preproc_ifdef",
    "struct_specifier",
    "template_declaration",
    "translation_unit",
    "type_definition",
    "union_specifier",
}

DOC = re.compile(rb"^(///(?!/)|//!|/\*\*(?![*/])|/\*!)")
TRAILING_DOC = re.compile(rb"^(///<|//!<|/\*\*<|/\*!<)")
NAMESPACE_END = re.compile(rb"^//\s*namespace(\s+[\w:]+)?\s*$")
DOXYGEN_GROUP = re.compile(r"^[@\\][{}]")
LEGAL = re.compile(r"copyright|spdx-license-identifier|licensed under|license:", re.IGNORECASE)
TODO = re.compile(r"\bTODO\b", re.IGNORECASE)
CPP_DIRECTIVE = re.compile(r"^(NOLINT|clang-format (off|on)\b|IWYU pragma:|cppcheck-suppress)")
PYTHON_DIRECTIVE = re.compile(r"^(noqa\b|type:\s*ignore|pylint:|fmt:\s*(off|on|skip)\b|isort:|pragma:|mypy:|pyright:|ruff:)")
PYTHON_ENCODING = re.compile(r"^#.*coding[:=]\s*[-\w.]+")
CPP_MARKER = re.compile(r"^(//+[!<]?|/\*+[!<]?)|\*+/$")
HUNK = re.compile(r"^@@ -\S+ \+(\d+)(?:,(\d+))? @@")
UNPARSABLE = re.compile(
    rb'"(?:\\.|[^"\\\n])*"|\'(?:\\.|[^\'\\\n])*\'|//[^\n]*|/\*[\s\S]*?\*/'
    rb"|\[\[[\s\S]*?\]\]|\bextern\s+template\b[^;]*;|\bDEPTHAI_(?:BEGIN|END)_SUPPRESS_DEPRECATION_WARNING\b"
)


@dataclass(frozen=True)
class Comment:
    line: int
    end_line: int
    column: int
    kind: str


@dataclass(frozen=True)
class Problem:
    path: str
    line: int
    column: int
    kind: str


def walk(node):
    stack = [node]
    while stack:
        node = stack.pop()
        yield node
        stack.extend(reversed(node.children))


def own_line(node, lines):
    row, column = node.start_point
    return not lines[row][:column].strip()


def ends_line(node, lines):
    row, column = node.end_point
    return not lines[row][column:].strip()


def cpp_body(node):
    return CPP_MARKER.sub("", node.text.decode("utf-8", "replace").strip()).strip()


def cpp_blocks(comments, lines):
    blocks = []
    for node in comments:
        if blocks:
            previous = blocks[-1][-1]
            if (
                own_line(previous, lines)
                and own_line(node, lines)
                and not CPP_DIRECTIVE.match(cpp_body(previous))
                and not CPP_DIRECTIVE.match(cpp_body(node))
                and previous.text.startswith(b"//")
                and node.text.startswith(b"//")
                and node.start_point.row == previous.end_point.row + 1
                and node.start_point.column == previous.start_point.column
                and bool(DOC.match(node.text)) == bool(DOC.match(previous.text))
            ):
                blocks[-1].append(node)
                continue
        blocks.append([node])
    return blocks


def member_access(body, member):
    access = "private" if body.parent is not None and body.parent.type == "class_specifier" else "public"
    for sibling in body.named_children:
        if sibling.start_byte >= member.start_byte:
            break
        if sibling.type == "access_specifier":
            access = sibling.text.decode()
    return access


def in_public_declaration_scope(node):
    child, parent = node, node.parent
    while parent is not None:
        if parent.type not in DECLARATION_SCOPES:
            return False
        if parent.type == "field_declaration_list" and member_access(parent, child) == "private":
            return False
        child, parent = parent, parent.parent
    return True


def next_declaration(node):
    sibling = node.next_named_sibling
    while sibling is not None and sibling.type == "comment":
        sibling = sibling.next_named_sibling
    return sibling


def follows(node, types):
    previous = node.prev_named_sibling
    return previous is not None and previous.type in types and previous.end_point.row == node.start_point.row


def cpp_kind(block, lines, reference):
    text = "\n".join(node.text.decode("utf-8", "replace") for node in block)
    first, last = block[0], block[-1]
    if LEGAL.search(text):
        return "legal"
    if all(CPP_DIRECTIVE.match(cpp_body(node)) for node in block):
        return "directive"
    if TODO.search(text):
        return "todo"
    if not own_line(first, lines):
        if NAMESPACE_END.match(first.text) and follows(first, {"namespace_definition"}):
            return "namespace end"
        if reference and TRAILING_DOC.match(first.text) and follows(first, DECLARATIONS) and in_public_declaration_scope(first):
            return "api doc"
        if ends_line(last, lines):
            return "trailing"
    elif reference and DOC.match(first.text) and not TRAILING_DOC.match(first.text):
        if DOXYGEN_GROUP.match(cpp_body(first)):
            return "api doc"
        declaration = next_declaration(last)
        if declaration is not None and declaration.type in DECLARATIONS and in_public_declaration_scope(first):
            return "api doc"
    return "doc" if DOC.match(first.text) else "comment"


def blank_unparsable(match):
    text = match[0]
    if text[:1] in (b'"', b"'", b"/"):
        return text
    return re.sub(rb"[^\n]", b" ", text)


def cpp_comments(data, reference):
    data = UNPARSABLE.sub(blank_unparsable, data)
    lines = data.split(b"\n")
    comments = [node for node in walk(PARSER.parse(data).root_node) if node.type == "comment"]
    for block in cpp_blocks(comments, lines):
        yield Comment(
            block[0].start_point.row + 1,
            block[-1].end_point.row + 1,
            block[0].start_point.column + 1,
            cpp_kind(block, lines, reference),
        )


def python_directive(row, comment):
    if row == 1 and comment.startswith("#!"):
        return True
    if row <= 2 and PYTHON_ENCODING.match(comment):
        return True
    return bool(PYTHON_DIRECTIVE.match(comment.lstrip("#").strip()))


def python_kind(block, trailing):
    text = "\n".join(comment for _, comment in block)
    if LEGAL.search(text):
        return "legal"
    if all(python_directive(row, comment) for row, comment in block):
        return "directive"
    if TODO.search(text):
        return "todo"
    return "trailing" if trailing else "comment"


def python_comments(data):
    source = data.decode("utf-8", "replace")
    blocks = []
    for token in tokenize.generate_tokens(io.StringIO(source).readline):
        if token.type != tokenize.COMMENT:
            continue
        row, column = token.start
        alone = not token.line[:column].strip() and not python_directive(row, token.string)
        if blocks:
            previous_row, previous_column, previous_alone, block = blocks[-1]
            if alone and previous_alone and row == previous_row + 1 and column == previous_column:
                block.append((row, token.string))
                blocks[-1] = (row, column, alone, block)
                continue
        blocks.append((row, column, alone, [(row, token.string)]))
    for last_row, column, alone, block in blocks:
        yield Comment(block[0][0], last_row, column + 1, python_kind(block, trailing=not alone))


def checked(path):
    suffix = Path(path).suffix
    parts = Path(path).parts
    return (
        suffix in CPP_SUFFIXES | PYTHON_SUFFIXES
        and not any(part.startswith(".") for part in parts)
        and not SKIPPED_FOLDERS.intersection(parts[:-1])
        and not path.startswith(SKIPPED_PREFIXES)
    )


def comments(path, data):
    suffix = Path(path).suffix
    if suffix in PYTHON_SUFFIXES:
        return python_comments(data)
    return cpp_comments(data, reference=path.startswith("include/") and suffix == ".hpp")


def find_problems(root, targets):
    problems = []
    count = 0
    for path, lines in sorted(targets.items()):
        file = root / path
        if not checked(path) or not file.is_file():
            continue
        count += 1
        try:
            found = list(comments(path, file.read_bytes()))
        except (SyntaxError, tokenize.TokenError) as error:
            problems.append(Problem(path, 1, 1, f"cannot read the Python tokens: {error}"))
            continue
        for comment in found:
            if comment.kind in ALLOWED_KINDS:
                continue
            if lines is not None and not any(line in lines for line in range(comment.line, comment.end_line + 1)):
                continue
            problems.append(Problem(path, comment.line, comment.column, comment.kind))
    return problems, count


def git(root, *args):
    result = subprocess.run(["git", "-C", str(root), *args], capture_output=True, check=True)
    return result.stdout.decode("utf-8", "replace")


def changed_lines(root, base):
    merge_base = git(root, "merge-base", base, "HEAD").strip()
    diff = git(
        root,
        "-c",
        "core.quotePath=false",
        "diff",
        "--unified=0",
        "--find-renames",
        "--no-color",
        "--no-ext-diff",
        "--src-prefix=a/",
        "--dst-prefix=b/",
        merge_base,
        "--",
    )
    targets = {}
    path = None
    header = False
    for line in diff.splitlines():
        if line.startswith("diff --git "):
            header, path = True, None
        elif header and line.startswith("+++ "):
            path = line[len("+++ b/"):] if line.startswith("+++ b/") else None
            if path is not None:
                targets.setdefault(path, set())
        elif line.startswith("@@ "):
            header = False
            match = HUNK.match(line)
            if path is not None and match:
                start = int(match[1])
                count = int(match[2]) if match[2] is not None else 1
                targets[path].update(range(start, start + count))
    for path in git(root, "ls-files", "--others", "--exclude-standard", "-z").split("\0"):
        if path:
            targets[path] = None
    return targets


def listed_files(root, paths):
    relative = [str(Path(path).resolve().relative_to(root.resolve())) for path in paths]
    files = git(root, "ls-files", "-z", "--cached", "--others", "--exclude-standard", "--", *relative).split("\0")
    return {path: None for path in files if path}


def hint():
    allowed = [
        "Doxygen comments (///, //!, /** */, /*! */, ///<) on public declarations in include/**/*.hpp",
        "the '} // namespace' comments of clang-format",
        "tool directives (NOLINT, clang-format off/on, noqa, type: ignore), license text",
    ]
    if "todo" in UNCHECKED_KINDS:
        allowed.append("TODO comments")
    if "trailing" in UNCHECKED_KINDS:
        allowed.append("comments at the end of a line of code")
    return (
        "Remove these comments. Make the code clear without them: names, functions, types, tests.\n"
        "Allowed: " + "; ".join(allowed) + "."
    )


def main(argv=None, root=ROOT):
    parser = argparse.ArgumentParser(
        description=(
            "Report comments in C, C++ and Python code. Without paths, check only the lines that changed "
            "since the merge base with --base, with uncommitted and untracked files. "
            "Exit code: 0 no problems, 1 problems, 2 the check could not run."
        )
    )
    parser.add_argument("--base", default="origin/develop", help="branch or commit to compare with (default: origin/develop)")
    parser.add_argument("paths", nargs="*", help="check all lines of these files or folders, not only the changed lines")
    args = parser.parse_args(argv)
    try:
        targets = listed_files(root, args.paths) if args.paths else changed_lines(root, args.base)
    except subprocess.CalledProcessError as error:
        print(f"check_comments: git failed: {error.stderr.decode('utf-8', 'replace').strip()}", file=sys.stderr)
        return 2
    except ValueError as error:
        print(f"check_comments: {error}", file=sys.stderr)
        return 2
    if args.paths and not targets:
        print(f"check_comments: no files match: {' '.join(args.paths)}", file=sys.stderr)
        return 2
    problems, count = find_problems(root, targets)
    for problem in problems:
        print(f"{problem.path}:{problem.line}:{problem.column}: {MESSAGES.get(problem.kind, problem.kind)}")
    if problems:
        print(f"\n{len(problems)} comments not allowed in {count} checked files.\n{hint()}", file=sys.stderr)
        return 1
    print(f"No comments to remove ({count} files checked).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
