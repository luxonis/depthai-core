"""Source-only regression tests for the binding CI gate; no DepthAI build needed."""

from contextlib import redirect_stderr, redirect_stdout
import io
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import check_python_bindings as checker


class SourceParsingTests(unittest.TestCase):
    def test_public_api_and_overloads(self):
        source = """
        namespace dai {
        class Example {
            void hidden();
        public:
            Example(int count = 1);
            Example(const Example&) = delete;
            Example& set(int value);
            Example& set(float value);
            int get() const { int local = 0; return local; }
            int x = 0, y = 0;
            struct Nested { enum class Mode { A, B [[deprecated]] }; };
        protected:
            struct Secret { int value; };
            void hiddenToo();
        };
        namespace detail { struct Hidden {}; }
        namespace internal { void hidden(); }
        namespace impl { void hidden(); }
        namespace { void hidden(); }
        void freeFunction(int value);
        }
        namespace dai::internal { struct AlsoHidden {}; }
        """
        api, errors = checker.public_api(source, "example.hpp")
        self.assertFalse(errors)
        self.assertEqual(set(api), {
            "dai::Example", "dai::Example::Example(int)",
            "dai::Example::set(int)", "dai::Example::set(float)",
            "dai::Example::get()const", "dai::Example::x", "dai::Example::y",
            "dai::Example::Nested", "dai::Example::Nested::Mode",
            "dai::Example::Nested::Mode::A", "dai::Example::Nested::Mode::B",
            "dai::freeFunction(int)",
        })

    def test_parameter_names_defaults_and_bodies_do_not_change_identity(self):
        before, _ = checker.public_api("namespace dai { void f(const X& x, int y = 1) {} }", "x.hpp")
        after, _ = checker.public_api("namespace dai { void f(const X& renamed, int z = 2) { other(); } }", "x.hpp")
        self.assertEqual(set(before), set(after))
        self.assertEqual(set(after), {"dai::f(constX&,int)"})

    def test_unnamed_enum_values_and_void_parameters(self):
        api, errors = checker.public_api("namespace dai { enum { A, B }; void f(void); }", "x.hpp")
        self.assertFalse(errors)
        self.assertEqual(set(api), {"dai::A", "dai::B", "dai::f()"})

    def test_preprocessor_branches_and_comments(self):
        source = """
        #define DECLARE(X) \\
            void X();
        namespace dai {
        // struct Fake {};
        struct Real {
        #ifdef OPTIONAL_FEATURE
            void optional();
        #else
            void fallback();
        #endif
        };
        }
        """
        api, errors = checker.public_api(source, "x.hpp")
        self.assertFalse(errors)
        self.assertEqual(set(api), {"dai::Real", "dai::Real::optional()", "dai::Real::fallback()"})

    def test_conditional_access_is_preserved_in_each_branch(self):
        source = '''namespace dai { class Example {
        #ifndef DEVICE
        public:
        #elif defined(OTHER)
        protected:
        #else
        private:
        #endif
            void added();
        private:
        #ifdef FEATURE
            public:
            #ifdef NESTED
                void nested();
            #else
                private:
            #endif
        #endif
            void sometimesPublic();
        private:
            void hidden();
        }; }'''
        api, errors = checker.public_api(source, "example.hpp")
        self.assertFalse(errors)
        self.assertEqual(set(api), {"dai::Example", "dai::Example::added()",
                                    "dai::Example::nested()", "dai::Example::sometimesPublic()"})

    def test_enum_targets_are_normalized_within_the_owning_enum(self):
        for target in ("CPP_NAME", "dai::Outer::CPP_NAME", "Outer::Mode::CPP_NAME", "Alias::CPP_NAME"):
            with self.subTest(target=target):
                evidence = checker.binding_evidence('''void bind() {
                    using namespace dai;
                    using Alias = dai::Outer::Mode;
                    py::enum_<Alias>(m, "Mode").value("renamed", %s);
                }''' % target)
                self.assertIn("dai::Outer::Mode::CPP_NAME", evidence)
        evidence = checker.binding_evidence('''void bind() {
            py::enum_<dai::Mode>(m, "Mode").value("CPP_NAME", dai::Other::CPP_NAME);
        }''')
        self.assertNotIn("dai::Mode::CPP_NAME", evidence)

    def test_binding_owners_aliases_lambdas_and_enums(self):
        source = """
        void bind() {
            using namespace dai;
            using Alias = dai::Example;
            py::class_<Alias> example(m, "Example");
            example.def(py::init<int>())
                .def("renamed", &Alias::original)
                .def("wrapped", [](Alias& self) { return self.originalWrapped(); })
                .def_readwrite("field", &Alias::field);
            py::enum_<Alias::Mode>(example, "Mode").value("A", Alias::Mode::A);
            py::class_<Other>(m, "Other").def("other", &Other::other);
            m.def("free", &dai::freeFunction);
            m.def("unqualified", &unqualified);
        }
        """
        evidence = checker.binding_evidence(source)
        for name in ("Example", "Example::Example", "Example::original", "Example::originalWrapped",
                     "Example::field", "Example::Mode", "Example::Mode::A", "Other::other", "freeFunction", "unqualified"):
            self.assertIn("dai::" + name, evidence)
        self.assertNotIn("dai::Example::other", evidence)

    def test_node_macros_and_function_scopes(self):
        source = """
        void bindA() {
            using namespace dai::node;
            auto node = ADD_NODE(First);
            node.def("first", &First::first);
        }
        void bindB() {
            using namespace dai::beta::node;
            auto node = ADD_BETA_NODE_DERIVED(Second, Base);
            node.def("second", &Second::second);
        }
        """
        evidence = checker.binding_evidence(source)
        self.assertIn("dai::node::First::first", evidence)
        self.assertIn("dai::beta::node::Second::second", evidence)
        self.assertNotIn("dai::node::First::second", evidence)

    def test_comments_docstrings_and_arbitrary_references_do_not_count(self):
        source = """
        void bind() {
            using namespace dai;
            py::class_<Example> example(m, "Example", DOC(dai, Example, missing));
            // example.def("missing", &Example::missing);
            auto unused = &Example::missing;
            example.def("exists", &Example::exists, DOC(dai, Example, missing));
        }
        """
        evidence = checker.binding_evidence(source)
        self.assertNotIn("dai::Example::missing", evidence)
        self.assertIn("dai::Example::exists", evidence)


class GateTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        root_patch = patch.object(checker, "ROOT", self.root)
        root_patch.start()
        self.addCleanup(root_patch.stop)
        self.header = "include/depthai/Example.hpp"
        self.binding = "bindings/python/src/ExampleBindings.cpp"
        self.write(self.header, "namespace dai { struct Example { void existing(); void legacyUnbound(); }; }")
        self.write(self.binding, 'void bind() { using namespace dai; py::class_<Example> ex(m, "Example"); ex.def("existing", &Example::existing); }')
        self.write("bindings/python/CMakeLists.txt", "set(SOURCE_LIST src/ExampleBindings.cpp)")
        self.write(checker.EXCEPTIONS, "{}")
        self.git("init", "-q")
        self.git("add", ".")
        self.git("-c", "user.name=Test", "-c", "user.email=test@example.invalid", "commit", "-qm", "baseline")

    def git(self, *args):
        return subprocess.check_output(["git", "-C", str(self.root), *args], stderr=subprocess.STDOUT)

    def write(self, path, content):
        target = self.root / path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(content)

    def check(self):
        output = io.StringIO()
        with redirect_stdout(output), redirect_stderr(output):
            failed = checker.check("HEAD")
        return failed, output.getvalue()

    def test_legacy_omissions_do_not_fail(self):
        self.write(self.header, (self.root / self.header).read_text() + "\n// documentation change\n")
        self.assertEqual(self.check()[0], False)

    def test_new_method_fails_until_bound(self):
        self.write(self.header, "namespace dai { struct Example { void existing(); void added(); }; }")
        failed, output = self.check()
        self.assertTrue(failed)
        self.assertIn("dai::Example::added()", output)
        self.write(self.binding, (self.root / self.binding).read_text().replace('ex.def("existing", &Example::existing)', 'ex.def("existing", &Example::existing).def("added", &Example::added)'))
        self.assertFalse(self.check()[0])

    def test_other_class_binding_cannot_satisfy_method(self):
        self.write(self.header, "namespace dai { struct Example { void added(); }; }")
        self.write(self.binding, 'void bind() { py::class_<dai::Other>(m, "Other").def("added", &dai::Other::added); }')
        self.assertTrue(self.check()[0])

    def test_lambda_only_counts_members_on_the_bound_instance(self):
        self.write(self.header, "namespace dai { struct Example { void added(); }; }")
        for expression in ("other.added()", "self.getOther().added()", "self.other.added()",
                           "[](dai::Other& self) { self.added(); }(other)"):
            with self.subTest(expression=expression):
                self.write(self.binding, '''void bind() {
                    py::class_<dai::Example> ex(m, "Example");
                    ex.def("added", [](dai::Example& self, dai::Other& other) { %s; });
                }''' % expression)
                self.assertTrue(self.check()[0])
        for parameter, expression in (("dai::Example& self", "self.added()"),
                                      ("dai::Example* self", "self->added()"),
                                      ("std::shared_ptr<dai::Example> self", "self->added()")):
            with self.subTest(parameter=parameter):
                self.write(self.binding, '''void bind() {
                    py::class_<dai::Example> ex(m, "Example");
                    ex.def("renamed", [](%s) { %s; });
                }''' % (parameter, expression))
                self.assertFalse(self.check()[0])

    def test_python_alias_does_not_bind_an_unrelated_cpp_member(self):
        self.write(self.header, "namespace dai { struct Example { void existing(); void added(); }; }")
        self.write(self.binding, '''void bind() {
            py::class_<dai::Example> ex(m, "Example");
            ex.def("added", &dai::Example::existing);
        }''')
        self.assertTrue(self.check()[0])

    def test_enum_binding_uses_the_cpp_enumerator(self):
        self.write(self.header, "namespace dai { enum class Mode { CPP_NAME }; }")
        self.write(self.binding, '''void bind() {
            using Mode = dai::Mode;
            py::enum_<Mode>(m, "Mode").value("PYTHON_NAME", Mode::CPP_NAME);
        }''')
        self.assertFalse(self.check()[0])
        self.write(self.header, "namespace dai { enum class Mode { CPP_NAME, PYTHON_NAME }; }")
        failed, output = self.check()
        self.assertTrue(failed)
        self.assertIn("dai::Mode::PYTHON_NAME", output)
        self.assertNotIn("dai::Mode::CPP_NAME: missing", output)

    def test_unscoped_enum_values_are_recognized(self):
        self.write(self.header, "namespace dai { enum Mode { CPP_NAME }; }")
        for target in ("CPP_NAME", "dai::CPP_NAME", "Mode::CPP_NAME", "dai::Mode::CPP_NAME"):
            with self.subTest(target=target):
                self.write(self.binding, '''void bind() {
                    using namespace dai;
                    py::enum_<Mode>(m, "Mode").value("PYTHON_NAME", %s);
                }''' % target)
                self.assertFalse(self.check()[0])

    def test_overload_requires_relevant_binding_update(self):
        self.write(self.header, "namespace dai { struct Example { void existing(); void existing(int value); }; }")
        failed, output = self.check()
        self.assertTrue(failed)
        self.assertIn("new overload/signature", output)
        # Whitespace and unrelated bindings are insufficient.
        self.write(self.binding, (self.root / self.binding).read_text().replace('ex.def(', '\n ex.def(').replace('); }', ').def("other", &Example::other); }'))
        self.assertTrue(self.check()[0])
        self.write(self.binding, (self.root / self.binding).read_text().replace('.def("other", &Example::other)', '.def("existing", py::overload_cast<int>(&Example::existing))'))
        self.assertFalse(self.check()[0])

    def test_new_overloads_need_distinct_applicable_bindings(self):
        self.write(self.header, "namespace dai { struct Example { void added(int); void added(float); }; }")
        self.write(self.binding, '''void bind() {
            py::class_<dai::Example> ex(m, "Example");
            ex.def("added", py::overload_cast<int>(&dai::Example::added));
        }''')
        self.assertTrue(self.check()[0])
        # A Python alias of the int overload still does not expose the float one.
        self.write(self.binding, (self.root / self.binding).read_text().replace(
            'ex.def("added",', 'ex.def("alias", py::overload_cast<int>(&dai::Example::added)); ex.def("added",'))
        self.assertTrue(self.check()[0])
        self.write(self.binding, (self.root / self.binding).read_text().replace(
            'ex.def("alias", py::overload_cast<int>', 'ex.def("alias", py::overload_cast<float>'))
        self.assertFalse(self.check()[0])

    def test_static_cast_of_old_overload_does_not_cover_new_overload(self):
        self.write(self.header, "namespace dai { struct Example { void existing(); void existing(int); }; }")
        self.write(self.binding, '''void bind() {
            py::class_<dai::Example> ex(m, "Example");
            ex.def("existing", static_cast<void (dai::Example::*)()>(&dai::Example::existing));
        }''')
        self.assertTrue(self.check()[0])
        self.write(self.binding, (self.root / self.binding).read_text().replace(
            'ex.def("existing",', 'ex.def("withValue", static_cast<void (dai::Example::*)(int value)>(&dai::Example::existing)); ex.def("existing",'))
        self.assertFalse(self.check()[0])

    def test_static_cast_const_and_free_function_signatures(self):
        self.write(self.header, '''namespace dai {
            struct Example { void added(const std::pair<int, float>& value) const; };
            void freeFunction(int value);
            void freeFunction(float value);
        }''')
        self.write(self.binding, '''void bind() {
            py::class_<dai::Example>(m, "Example").def("added",
                static_cast<void (dai::Example::*)(const std::pair<int, float>& value) const>(&dai::Example::added));
            m.def("freeFunction", static_cast<void (*)(int)>(&dai::freeFunction));
        }''')
        failed, output = self.check()
        self.assertTrue(failed)
        self.assertIn("dai::freeFunction(float)", output)
        self.assertNotIn("dai::Example::added", output)
        self.write(self.binding, (self.root / self.binding).read_text().replace(
            'm.def("freeFunction",', 'm.def("floatFunction", static_cast<void (*)(float)>(&dai::freeFunction)); m.def("freeFunction",'))
        self.assertFalse(self.check()[0])

    def test_unresolved_explicit_cast_cannot_cover_new_overload(self):
        self.write(self.header, "namespace dai { struct Example { void existing(); void existing(int); }; }")
        self.write(self.binding, '''void bind() {
            using Callback = void (dai::Example::*)();
            py::class_<dai::Example>(m, "Example").def("existing", static_cast<Callback>(&dai::Example::existing));
        }''')
        self.assertTrue(self.check()[0])

    def test_conditionally_public_method_needs_binding(self):
        self.write(self.header, '''namespace dai { struct Example {
            void existing();
        #ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
            public:
        #else
            private:
        #endif
            void added();
        }; }''')
        failed, output = self.check()
        self.assertTrue(failed)
        self.assertIn("dai::Example::added()", output)
        self.write(self.binding, (self.root / self.binding).read_text().replace(
            '&Example::existing)', '&Example::existing).def("added", &Example::added)'))
        self.assertFalse(self.check()[0])

    def test_unsupported_conditional_declaration_fails_closed(self):
        self.write(self.header, '''namespace dai { void added(
        #ifdef FEATURE
            float value
        #else
            int value
        #endif
        ); }''')
        failed, output = self.check()
        self.assertTrue(failed)
        self.assertIn("cannot inspect new C++ syntax", output)

    def test_new_constructor_overloads_need_distinct_bindings(self):
        self.write(self.header, "namespace dai { struct Example { Example(int); Example(float); }; }")
        self.write(self.binding, '''void bind() {
            py::class_<dai::Example>(m, "Example").def(py::init<int>());
        }''')
        self.assertTrue(self.check()[0])
        self.write(self.binding, (self.root / self.binding).read_text().replace(
            '.def(py::init<int>())', '.def(py::init<int>()).def(py::init<float>())'))
        self.assertFalse(self.check()[0])

    def test_const_and_template_overload_signatures(self):
        self.write(self.header, '''namespace dai { struct Example {
            void added(std::pair<int, float> value);
            void added(std::pair<int, float> value) const;
        }; }''')
        self.write(self.binding, '''void bind() {
            py::class_<dai::Example> ex(m, "Example");
            ex.def("added", py::overload_cast<std::pair<int, float>>(&dai::Example::added, py::const_));
        }''')
        self.assertTrue(self.check()[0])
        self.write(self.binding, (self.root / self.binding).read_text().replace(
            'ex.def("added",', 'ex.def("mutable", py::overload_cast<std::pair<int, float>>(&dai::Example::added)); ex.def("added",'))
        self.assertFalse(self.check()[0])

    def test_consolidated_overloads_need_an_exact_exception(self):
        self.write(self.header, "namespace dai { struct Example { void added(int); void added(float); }; }")
        self.write(self.binding, '''void bind() {
            py::class_<dai::Example> ex(m, "Example");
            ex.def("added", [](dai::Example& self, py::object value) { self.added(convert(value)); });
        }''')
        self.assertTrue(self.check()[0])
        self.write(checker.EXCEPTIONS, '{"dai::Example::added(float)": "Handled by the shared Python dispatcher"}')
        self.assertFalse(self.check()[0])

    def test_new_untracked_header_and_cmake_registration(self):
        self.write("include/depthai/New.hpp", "namespace dai { struct New {}; }")
        self.write("bindings/python/src/NewBindings.cpp", 'void bindNew() { py::class_<dai::New>(m, "New"); }')
        self.assertTrue(self.check()[0])
        self.write("bindings/python/CMakeLists.txt", "set(SOURCE_LIST src/ExampleBindings.cpp src/NewBindings.cpp)")
        self.assertFalse(self.check()[0])

    def test_exact_exception_does_not_hide_future_members(self):
        self.write(self.header, "namespace dai { struct Example { void added(); }; }")
        self.write(checker.EXCEPTIONS, '{"dai::Example::added()": "Native C++ integration only"}')
        self.assertFalse(self.check()[0])
        self.write(self.header, "namespace dai { struct Example { void added(); void added(int); }; }")
        self.assertTrue(self.check()[0])
        self.write(checker.EXCEPTIONS, '{"dai::Example::*": "Too broad"}')
        self.assertTrue(self.check()[0])  # Keys are literal; '*' never acts as a wildcard.

    def test_exact_pointer_exception_is_accepted(self):
        self.write(self.header, "namespace dai { struct Example { void setExternalMemory(void* ptr); }; }")
        self.write(checker.EXCEPTIONS, '{"dai::Example::setExternalMemory(void*)": "Native C++ integration only"}')
        self.assertFalse(self.check()[0])
        self.write(self.header, "namespace dai { struct Example { void setExternalMemory(const void* ptr); }; }")
        self.assertTrue(self.check()[0])

    def test_exception_requires_a_reason(self):
        for value in ('{"dai::Example::existing()": " "}', '{"dai::Example::existing()": null}', '[]'):
            with self.subTest(value=value):
                self.write(checker.EXCEPTIONS, value)
                with self.assertRaises(ValueError):
                    self.check()

    def test_moving_header_does_not_introduce_new_api(self):
        (self.root / self.header).rename(self.root / "include/depthai/Renamed.hpp")
        self.assertFalse(self.check()[0])

    def test_malformed_header_fails(self):
        self.write("include/depthai/New.hpp", "namespace dai { struct New { @@@ void method(); }; }")
        failed, output = self.check()
        self.assertTrue(failed)
        self.assertIn("cannot inspect new C++ syntax", output)


if __name__ == "__main__":
    unittest.main()
