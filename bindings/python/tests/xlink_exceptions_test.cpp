#include "depthai_pybind11_tests.hpp"

#include "depthai/depthai.hpp"

TEST_SUBMODULE(xlink_exceptions, m) {

#ifdef __APPLE__
    // Both extension bundles statically link the SDK. On macOS their strong
    // exception RTTI symbols remain distinct, so a translator in depthai cannot
    // catch exceptions thrown by this bundle. Reuse its Python exception types
    // while catching the helper's native types locally.
    py::register_local_exception_translator([](std::exception_ptr error) {
        try {
            if(error) std::rethrow_exception(error);
        } catch(const dai::XLinkWriteError& ex) {
            py::set_error(py::module_::import("depthai").attr("XLinkWriteError"), ex.what());
        } catch(const dai::XLinkReadError& ex) {
            py::set_error(py::module_::import("depthai").attr("XLinkReadError"), ex.what());
        } catch(const dai::XLinkError& ex) {
            py::set_error(py::module_::import("depthai").attr("XLinkError"), ex.what());
        }
    });
#endif

    m.def("throw_xlink_error", [](){
        throw dai::XLinkError(X_LINK_ERROR, "stream", "Yikes!");
    });

    m.def("throw_xlink_read_error", [](){
        throw dai::XLinkReadError(X_LINK_ERROR, "stream_read");
    });

    m.def("throw_xlink_write_error", [](){
        throw dai::XLinkWriteError(X_LINK_ERROR, "stream_write");
    });

}
