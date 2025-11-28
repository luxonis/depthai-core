Examples of how to use target built by CMake configuration `DEPTHAI_PYTHON_EMBEDDED_MODULE_TARGET`, which is installed as a separate shared lib(dependent on core).

This allows other C++ projects to include the additional lib and use python to build its pipeline.

This example set encompasses 2 scenarios:
- build_pipeline_from_script.cpp: Embedding python in C++ projects and using it to build the pipeline only, then disgarding the interpreter. Note that this does not allow depthai_nodes or ANY other python-derived classes to survive and be used.
- TODO: Embedding python in C++ projects and using it to build the pipeline *and* keep it running such that it can be used to run depthai_nodes and other python-derived classes. Note that this has a performance penalty because python is slow.