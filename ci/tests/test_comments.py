from contextlib import redirect_stderr, redirect_stdout
import io
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import textwrap
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import check_comments as checker


def kinds(source, path="src/example.cpp"):
    data = textwrap.dedent(source).encode()
    return [(comment.line, comment.kind) for comment in checker.comments(path, data)]


class CppCommentTests(unittest.TestCase):
    def test_plain_and_trailing_comments(self):
        source = """\
            // explains the function
            void f() {
                g();  // wake the node
                /* block */
                h(/*enabled=*/true);  /* why */
            }
            """
        self.assertEqual(kinds(source), [(1, "comment"), (3, "trailing"), (4, "comment"), (5, "comment"), (5, "trailing")])

    def test_consecutive_line_comments_are_one_block(self):
        source = """\
            // one
            // two

            // three
            int x;
            """
        comments = list(checker.comments("src/a.cpp", textwrap.dedent(source).encode()))
        self.assertEqual([(c.line, c.end_line) for c in comments], [(1, 2), (4, 4)])

    def test_namespace_end_comments_of_clang_format(self):
        source = """\
            namespace dai {
            namespace node {
            int x;  // namespace dai
            }  // namespace node
            }  // namespace dai
            """
        self.assertEqual(kinds(source), [(3, "trailing"), (4, "namespace end"), (5, "namespace end")])

    def test_doc_comments_on_public_declarations_in_include(self):
        source = """\
            namespace dai {
            /** A class */
            class Example {
               public:
                /// Sets a value.
                void set(int value);
                int count = 0;  ///< Number of items.
                enum class Mode {
                    /** First mode */
                    A,
                    B,  ///< Second mode
                };
               private:
                /// Hidden helper.
                void helper();
                /** Hidden field */
                int hidden = 0;
            };
            /** Free function */
            void free();
            }  // namespace dai
            """
        expected = [
            (2, "api doc"),
            (5, "api doc"),
            (7, "api doc"),
            (9, "api doc"),
            (11, "api doc"),
            (14, "doc"),
            (16, "doc"),
            (19, "api doc"),
            (21, "namespace end"),
        ]
        self.assertEqual(kinds(source, "include/depthai/Example.hpp"), expected)
        outside = [(line, {"api doc": "trailing" if line in (7, 11) else "doc"}.get(kind, kind)) for line, kind in expected]
        self.assertEqual(kinds(source, "src/Example.hpp"), outside)
        self.assertEqual(kinds(source, "include/depthai/Example.h"), outside)

    def test_struct_members_are_public_and_class_members_private(self):
        source = """\
            struct S {
                /// doc
                int a;
            };
            class C {
                /// doc
                int b;
            };
            class Outer {
                class Inner {
                   public:
                    /// doc
                    void f();
                };
            };
            """
        self.assertEqual(kinds(source, "include/depthai/S.hpp"), [(2, "api doc"), (6, "doc"), (12, "doc")])

    def test_plain_comment_above_public_declaration(self):
        self.assertEqual(kinds("// Sets a value.\nvoid set(int value);\n", "include/depthai/A.hpp"), [(1, "comment")])

    def test_doc_comments_that_are_not_on_a_declaration(self):
        source = """\
            /** @file Example.hpp */
            #pragma once
            namespace dai {
            inline void f() {
                /// inside a body
                int x = 0;
            }
            class C {
               public:
                /** Orphan before an access specifier */
               private:
                int y;
            };
            void g(int a /**< parameter */);
            /** @}  End of group */
            }
            """
        expected = [(1, "doc"), (5, "doc"), (10, "doc"), (14, "doc"), (15, "api doc")]
        self.assertEqual(kinds(source, "include/depthai/C.hpp"), expected)

    def test_attributes_and_extern_templates_do_not_break_attachment(self):
        source = """\
            namespace dai {
            enum class Sensor : int {
                /** New name */
                NEW = 1,
                /** @deprecated Use NEW */
                OLD [[deprecated("Use NEW")]] = NEW,
                /** Another */
                OTHER = 2,
            };
            }  // namespace dai
            extern template struct dai::Serializable<dai::Sensor>;
            """
        expected = [(3, "api doc"), (5, "api doc"), (7, "api doc"), (10, "namespace end")]
        self.assertEqual(kinds(source, "include/depthai/Sensor.hpp"), expected)

    def test_directives_license_text_and_todo(self):
        source = """\
            // Copyright 2024 Example
            // Licensed under the MIT License.
            int a;  // NOLINT(readability-magic-numbers)
            // clang-format off
            int b;
            // clang-format on
            // TODO: remove after the next release
            //       when the firmware supports it
            void h(int /* unused */);
            """
        expected = [(1, "legal"), (3, "directive"), (4, "directive"), (6, "directive"), (7, "todo"), (9, "comment")]
        self.assertEqual(kinds(source), expected)

    def test_strings_are_not_comments(self):
        source = 'const char* url = "http://example.com // no";\nconst char* c = "/* no */";\n'
        self.assertEqual(kinds(source), [])


class PythonCommentTests(unittest.TestCase):
    def test_python_comments(self):
        source = '''\
            #!/usr/bin/env python3
            # -*- coding: utf-8 -*-
            """A docstring is not a comment."""
            import os  # noqa: F401
            value = "# not a comment"
            # explains the next line
            # and continues here
            run()  # trailing
            x = call()  # type: ignore[attr-defined]
            # TODO: rename
            '''
        expected = [(1, "directive"), (2, "directive"), (4, "directive"), (6, "comment"), (8, "trailing"), (9, "directive"), (10, "todo")]
        self.assertEqual(kinds(source, "scripts/tool.py"), expected)


class PathTests(unittest.TestCase):
    def test_checked_paths(self):
        skipped = [
            "tests/src/a.cpp",
            "examples/python/a.py",
            "src/3rdparty/x.hpp",
            "bindings/python/external/x.h",
            "shared/depthai-shared/x.hpp",
            "ci/tests/test_comments.py",
            "src/bspatch/bspatch.c",
            "include/depthai/utility/span.hpp",
            "device/rvc4/src/vendor/dai_device_common.h",
            "CMakeLists.txt",
            ".github/workflows/main.workflow.yml",
            "ci/check_format.sh",
            ".gitconfig",
            ".ycm_extra_conf.py",
            ".github/scripts/a.py",
            "device/rvc4/.devcontainer/setup.py",
            "src/.cache/a.cpp",
        ]
        checked = [
            "src/pipeline/Node.cpp",
            "include/depthai/A.hpp",
            "ci/check_comments.py",
            "device/rvc4/src/main.cpp",
            "device/rvc4/src/vendor_bridge.cpp",
            "src/tests_helper.cpp",
        ]
        self.assertEqual([path for path in skipped if checker.checked(path)], [])
        self.assertEqual([path for path in checked if not checker.checked(path)], [])


class CommandLineTests(unittest.TestCase):
    def setUp(self):
        environment = patch.dict(os.environ, {"GIT_CONFIG_GLOBAL": os.devnull, "GIT_CONFIG_NOSYSTEM": "1"})
        environment.start()
        self.addCleanup(environment.stop)
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        self.git("init", "-q")
        self.write("src/a.cpp", "// old comment\nint a = 1;\nint b = 2;  // old trailing\nint c = 3;\n")
        self.write("src/moved.cpp", "// keep\nint m;\n")
        self.commit("base")
        self.git("tag", "base")

    def tearDown(self):
        self.directory.cleanup()

    def git(self, *args):
        subprocess.run(
            ["git", "-C", str(self.root), "-c", "init.defaultBranch=main", "-c", "user.name=test", "-c", "user.email=test@example.com",
             "-c", "commit.gpgsign=false", "-c", "tag.gpgsign=false", "-c", "core.hooksPath=/dev/null",
             "-c", "maintenance.auto=false", "-c", "gc.auto=0", *args],
            check=True,
            capture_output=True,
        )

    def write(self, path, text):
        file = self.root / path
        file.parent.mkdir(parents=True, exist_ok=True)
        file.write_text(text)

    def commit(self, message):
        self.git("add", "-A")
        self.git("commit", "-q", "-m", message)

    def run_checker(self, *args):
        out, err = io.StringIO(), io.StringIO()
        with redirect_stdout(out), redirect_stderr(err):
            code = checker.main(list(args), root=self.root)
        return code, out.getvalue().splitlines(), err.getvalue()

    def test_only_changed_lines_are_reported(self):
        self.write("src/a.cpp", "// old comment\nint a = 1;\nint b = 20;  // old trailing\nint c = 3;\n// new comment\nint d = 4;\n")
        self.git("mv", "src/moved.cpp", "src/renamed.cpp")
        self.commit("change")
        with open(self.root / "src/a.cpp", "a") as file:
            file.write("int e;  // uncommitted\n")
        self.write("scripts/new.py", "# untracked comment\nx = 1\n")
        with patch.object(checker, "ALLOWED_KINDS", checker.ALLOWED_KINDS - {"trailing"}):
            code, out, err = self.run_checker("--base", "base")
        self.assertEqual(code, 1)
        self.assertEqual(out, [
            "scripts/new.py:1:1: comment not allowed",
            "src/a.cpp:3:14: trailing comment not allowed",
            "src/a.cpp:5:1: comment not allowed",
            "src/a.cpp:7:9: trailing comment not allowed",
        ])
        self.assertIn("Remove these comments", err)

    def test_change_without_comments_passes(self):
        with open(self.root / "src/a.cpp", "a") as file:
            file.write("int z = 4;\n")
        self.commit("change")
        code, out, _ = self.run_checker("--base", "base")
        self.assertEqual((code, out), (0, ["No comments to remove (1 files checked)."]))

    def test_paths_check_all_lines(self):
        code, out, _ = self.run_checker(str(self.root / "src/a.cpp"))
        self.assertEqual(code, 1)
        self.assertEqual(out, ["src/a.cpp:1:1: comment not allowed"])

    def test_unchecked_kinds_can_be_checked_later(self):
        self.write("src/later.cpp", "// TODO: rename\nint t;  // trailing\n")
        self.assertEqual(self.run_checker("--base", "base")[0], 0)
        for kind, problem in (("todo", "src/later.cpp:1:1: TODO comment not allowed"), ("trailing", "src/later.cpp:2:9: trailing comment not allowed")):
            with self.subTest(kind=kind), patch.object(checker, "ALLOWED_KINDS", checker.ALLOWED_KINDS - {kind}):
                self.assertEqual(self.run_checker("--base", "base")[:2], (1, [problem]))

    def test_errors_exit_with_two(self):
        self.assertEqual(self.run_checker("--base", "missing")[0], 2)
        self.assertEqual(self.run_checker(str(self.root / "nothing"))[0], 2)


if __name__ == "__main__":
    unittest.main()
