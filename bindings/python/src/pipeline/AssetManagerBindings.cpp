#include "AssetManagerBindings.hpp"

// depthai
#include "depthai/pipeline/AssetManager.hpp"

void AssetManagerBindings::bind(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    // Type definitions
    py::class_<Asset, std::shared_ptr<Asset>> asset(m, "Asset", DOC(dai, Asset));
    py::class_<AssetManager> assetManager(m, "AssetManager", DOC(dai, AssetManager));

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    // Bind Asset
    asset.def("getData", [](Asset& self) { return self.getData(); }, DOC(dai, Asset, getData));

    asset.def("setData", &dai::Asset::setData, py::arg("data"), DOC(dai, Asset, setData))
        .def("getSize", &dai::Asset::getSize, DOC(dai, Asset, getSize))
        .def("getRelativeUri", &dai::Asset::getRelativeUri, DOC(dai, Asset, getRelativeUri))
        .def("setFile", &dai::Asset::setFile, py::arg("path"), py::arg("size"), DOC(dai, Asset, setFile));

    asset.def(py::init<>())
        .def(py::init<std::string>())
        .def_readonly("key", &Asset::key)
        // Mutable view; replacing the asset storage invalidates existing views.
        .def_property(
            "data",
            [](py::object& obj) {
                dai::Asset& a = obj.cast<dai::Asset&>();
                auto& data = a.getData();
                return py::array_t<std::uint8_t>(data.size(), data.data(), obj);
            },
            [](py::object& obj, py::array_t<std::uint8_t, py::array::c_style> array) {
                dai::Asset& a = obj.cast<dai::Asset&>();
                a.setData({array.data(), array.data() + array.size()});
            },
            "Mutable view of asset data. Replacing the storage invalidates existing views; use getData() for an owned copy.")
        .def_readwrite("alignment", &Asset::alignment);

    // Bind AssetManager
    assetManager.def("setLazy", &AssetManager::setLazy, py::arg("key"), py::arg("path"), py::arg("alignment") = 64, DOC(dai, AssetManager, setLazy))
        .def("getSerializedSize", &AssetManager::getSerializedSize, py::arg("offset") = 0, DOC(dai, AssetManager, getSerializedSize));

    assetManager.def("setRootPath", &dai::AssetManager::setRootPath, py::arg("rootPath"), DOC(dai, AssetManager, setRootPath));

    assetManager.def(py::init<>())
        .def(py::init<std::string>())
        .def("addExisting", &AssetManager::addExisting, py::arg("assets"), DOC(dai, AssetManager, addExisting))
        .def("set", static_cast<std::shared_ptr<dai::Asset> (AssetManager::*)(Asset)>(&AssetManager::set), py::arg("asset"), DOC(dai, AssetManager, set))
        .def("set",
             static_cast<std::shared_ptr<dai::Asset> (AssetManager::*)(const std::string&, Asset)>(&AssetManager::set),
             py::arg("key"),
             py::arg("asset"),
             DOC(dai, AssetManager, set, 2))
        .def("set",
             static_cast<std::shared_ptr<dai::Asset> (AssetManager::*)(const std::string& key, const std::filesystem::path& path, int alignment)>(
                 &AssetManager::set),
             py::arg("key"),
             py::arg("path"),
             py::arg("alignment") = 64,
             DOC(dai, AssetManager, set, 3))
        .def("set",
             static_cast<std::shared_ptr<dai::Asset> (AssetManager::*)(const std::string& key, const std::vector<std::uint8_t>& data, int alignment)>(
                 &AssetManager::set),
             py::arg("key"),
             py::arg("data"),
             py::arg("alignment") = 64,
             DOC(dai, AssetManager, set, 4))
        .def("get",
             static_cast<std::shared_ptr<const Asset> (AssetManager::*)(const std::string&) const>(&AssetManager::get),
             py::arg("key"),
             DOC(dai, AssetManager, get))
        .def("get",
             static_cast<std::shared_ptr<Asset> (AssetManager::*)(const std::string&)>(&AssetManager::get),
             py::arg("key"),
             DOC(dai, AssetManager, get, 2))
        .def("getAll", static_cast<std::vector<std::shared_ptr<const Asset>> (AssetManager::*)() const>(&AssetManager::getAll), DOC(dai, AssetManager, getAll))
        .def("getAll", static_cast<std::vector<std::shared_ptr<Asset>> (AssetManager::*)()>(&AssetManager::getAll), DOC(dai, AssetManager, getAll, 2))
        .def("size", &AssetManager::size, DOC(dai, AssetManager, size))
        .def("remove", &AssetManager::remove, py::arg("key"), DOC(dai, AssetManager, remove))
        .def("getRootPath", &AssetManager::getRootPath, DOC(dai, AssetManager, getRootPath));
}
