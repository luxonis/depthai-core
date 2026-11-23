"""Device-free checks of NNArchive runtime dependencies in the Python API."""

import io
import json
import tarfile

import depthai as dai

REGISTRATION = "runtime/libTestReg.so"
IMPLEMENTATION = "runtime/libTestImplDsp.so"


def _write_archive(path):
    config = {
        "config_version": "1.1",
        "model": {
            "metadata": {"name": "udo-test", "path": "model.dlc"},
            "inputs": [],
            "outputs": [],
            "runtime_dependencies": {
                "snpe_udo": [{"registration": REGISTRATION, "implementations": [{"runtime": "dsp", "path": IMPLEMENTATION}]}]
            },
        },
    }
    members = {
        "config.json": json.dumps(config).encode(),
        "model.dlc": b"dlc",
        REGISTRATION: b"registration\x00\xff",
        IMPLEMENTATION: b"implementation",
    }
    with tarfile.open(path, "w") as archive:
        for name, data in members.items():
            info = tarfile.TarInfo(name)
            info.size = len(data)
            archive.addfile(info, io.BytesIO(data))


def test_runtime_libraries_are_bytes(tmp_path):
    path = tmp_path / "udo.tar"
    _write_archive(path)
    archive = dai.NNArchive(str(path))

    libraries = archive.getRuntimeLibraries()
    assert libraries == {REGISTRATION: b"registration\x00\xff", IMPLEMENTATION: b"implementation"}
    assert all(type(data) is bytes for data in libraries.values())

    package = archive.getConfigV1().model.runtimeDependencies.snpeUdo[0]
    assert package.registration == REGISTRATION
    assert [(i.runtime, i.path) for i in package.implementations] == [("dsp", IMPLEMENTATION)]


def test_udo_packages_are_editable_in_network_properties():
    package = dai.NeuralNetworkProperties.UdoPackage()
    package.registration = REGISTRATION
    package.dspImplementations = [IMPLEMENTATION]

    properties = dai.NeuralNetworkProperties()
    assert len(properties.udoPackages) == 0
    assert properties.udoLibraryUris == {}
    properties.udoPackages.append(package)
    properties.udoPackages[0].dspImplementations = [IMPLEMENTATION, "runtime/libOther.so"]
    properties.udoLibraryUris = {REGISTRATION: "asset:registration"}
    assert properties.udoPackages[0].dspImplementations == [IMPLEMENTATION, "runtime/libOther.so"]
    assert properties.udoLibraryUris == {REGISTRATION: "asset:registration"}
    properties.udoPackages = [package]
    assert properties.udoPackages[0].dspImplementations == [IMPLEMENTATION]
