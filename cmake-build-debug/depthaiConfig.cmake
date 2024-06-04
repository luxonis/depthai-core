# Get whether library was build as shared or not
set(DEPTHAI_SHARED_LIBS )

# Get library options
# Get whether library was build with Backward or not
set(DEPTHAI_ENABLE_BACKWARD ON)
set(DEPTHAI_XLINK_LOCAL )

# Specify that this is config mode (Called by find_package)
set(CONFIG_MODE TRUE)

# Compute the installation prefix relative to this file.
set(_IMPORT_PREFIX "../../../../.hunter/_Base/1292e4d/2d438b4/d8eb9c7/Install")

# Add dependencies file
include("${CMAKE_CURRENT_LIST_DIR}/depthaiDependencies.cmake")

# Add the targets file
include("${CMAKE_CURRENT_LIST_DIR}/depthaiTargets.cmake")

# Cleanup
set(_IMPORT_PREFIX)
