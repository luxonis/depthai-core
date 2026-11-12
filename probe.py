import pathlib
import subprocess
import sys

if sys.argv[1] == 'gate':
    script = pathlib.Path('gate.cmake')
    script.write_text('set(WIN32 TRUE)\nset(CMAKE_SIZEOF_VOID_P 8)\nset(DEPTHAI_ENABLE_APRIL_TAG ON CACHE BOOL "" FORCE)\ninclude("${CMAKE_CURRENT_LIST_DIR}/sdk/cmake/depthaiOptions.cmake")\nif(DEPTHAI_HAS_APRIL_TAG)\nmessage(FATAL_ERROR "Expected Windows gate was not observed")\nendif()\nmessage(STATUS "CONFIRMED: DEPTHAI_ENABLE_APRIL_TAG=ON but DEPTHAI_HAS_APRIL_TAG=OFF before any dependency download")\n')
    result = subprocess.run(['cmake', '-P', str(script)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    pathlib.Path('gate-result.txt').write_text(result.stdout)
    print(result.stdout)
    sys.exit(result.returncode)
elif sys.argv[1] == 'enable':
    options = pathlib.Path('sdk/cmake/depthaiOptions.cmake')
    original = options.read_text()
    gate = 'if(WIN32)\n    message(STATUS "AprilTag node is not supported on Windows")\n    set(DEPTHAI_HAS_APRIL_TAG OFF)\nendif()'
    assert original.count(gate) == 1
    options.write_text(original.replace(gate, '# Windows feature gate removed for diagnosis only'))
