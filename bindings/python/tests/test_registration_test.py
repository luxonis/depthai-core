"""Check optional Python test registration without building the SDK."""

import subprocess
from pathlib import Path

import pytest


@pytest.mark.parametrize("curl_enabled", ["ON", "OFF"])
@pytest.mark.parametrize("gil_free", ["ON", "OFF"])
def test_model_zoo_registration_requires_curl(tmp_path, curl_enabled, gil_free):
    """Keep HTTP regressions out of builds that do not provide model-zoo I/O."""
    script = tmp_path / "check_registration.cmake"
    # Evaluate the real registration file. Stub only its build-target commands;
    # this check needs the selected test lists, not compiled SDK dependencies.
    script.write_text('''
cmake_policy(SET CMP0057 NEW)
function(pybind11_add_module)
endfunction()
function(add_custom_target)
endfunction()
function(target_link_libraries)
endfunction()
function(target_compile_definitions)
endfunction()
function(add_test)
endfunction()
function(set_tests_properties)
endfunction()
include("${TEST_DIRECTORY}/CMakeLists.txt")
foreach(test_list IN ITEMS PYBIND11_TEST_FILES PYBIND11_PYTEST_FILES)
    if(DEPTHAI_ENABLE_CURL)
        if(NOT "model_zoo_gil_test.py" IN_LIST ${test_list})
            message(FATAL_ERROR "Model-zoo test missing with libcurl enabled")
        endif()
    elseif("model_zoo_gil_test.py" IN_LIST ${test_list})
        message(FATAL_ERROR "Model-zoo test registered without libcurl")
    endif()
    if(NOT "nn_archive_gil_test.py" IN_LIST ${test_list})
        message(FATAL_ERROR "Archive test must not depend on libcurl")
    endif()
    if(DEPTHAI_PYTHON_GIL_FREE)
        if(NOT "freethreading_test.py" IN_LIST ${test_list})
            message(FATAL_ERROR "Free-threading tests missing from GIL-free build")
        endif()
    elseif("freethreading_test.py" IN_LIST ${test_list})
        message(FATAL_ERROR "Free-threading tests registered without a GIL-free build")
    endif()
endforeach()
''')
    result = subprocess.run(
        [
            "cmake",
            f"-DDEPTHAI_ENABLE_CURL={curl_enabled}",
            f"-DDEPTHAI_PYTHON_GIL_FREE={gil_free}",
            f"-DTEST_DIRECTORY={Path(__file__).resolve().parent}",
            "-P", str(script),
        ],
        capture_output=True,
        text=True,
        timeout=15,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
