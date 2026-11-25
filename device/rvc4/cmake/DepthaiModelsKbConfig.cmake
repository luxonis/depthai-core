# Configuration for the device side model artifacts.
#
# The models are served by Artifact Keeper and read anonymously:
#   snapshot -> luxonis-rvc4-snapshot-local
#   release  -> luxonis-rvc4-release-local
# See cmake/DepthaiModelsKbDownloader.cmake.

# Maturity level "snapshot" / "release"
set(DEPTHAI_MODELS_KB_MATURITY "snapshot")

# Path under the repository, without a leading slash.
# Layout: luxonis-rvc4-snapshot-local/tof_fusion/<file>
set(DEPTHAI_MODELS_KB_PATH_PREFIX "tof_fusion")

# Version or commit directory. Leave empty to add no directory level.
# The files sit directly under tof_fusion/, so no version directory is used.
set(DEPTHAI_MODELS_KB_VERSION "")

# ToF neural fusion model, XOR'd. Feeds DeviceModelZoo::TOF_NEURAL_FUSION_672X804.
set(DEPTHAI_MODELS_KB_TOF_FUSION_NAME "TOF_NEURAL_FUSION_804x672_M.dlc")
set(DEPTHAI_MODELS_KB_TOF_FUSION_SHA256 "55df57c759b96c76d168fc86fc1068a597d0883b51a3cf22b61191c1c67dd750")
