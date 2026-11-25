# Entry point of the Hexagon SDK make system for this DSP module. build.sh runs it as
#   make V=hexagon_Release_toolv87_v73 BUILD_DIR=<output directory> dlls
# after it sourced <HEXAGON_SDK_ROOT>/setup_sdk_env.source. The SDK make fragments do the
# work; this file checks the environment and includes them in the order the SDK requires.

# setup_sdk_env.source exports SDK_SETUP_ENV together with HEXAGON_SDK_ROOT and the tool paths.
ifndef SDK_SETUP_ENV
$(error The Hexagon SDK environment is not set: source <HEXAGON_SDK_ROOT>/setup_sdk_env.source first)
endif

include $(HEXAGON_SDK_ROOT)/build/make.d/utils.min

# V names the build variant as <target>_<configuration>_<toolchain>_<architecture>. The target
# selects the <target>.min and <target>_deps.min files of this directory; only "hexagon" exists.
ifndef V
$(error V is not set: pass the build variant, for example V=hexagon_Release_toolv87_v73)
endif
V_TARGET = $(word 1,$(subst _, ,$(V)))
ifeq ($(wildcard $(V_TARGET).min $(V_TARGET)_deps.min),)
$(error Target "$(V_TARGET)" of variant "$(V)" is not supported: $(V_TARGET).min and $(V_TARGET)_deps.min are missing in $(CURDIR))
endif

BUILD_OUTPUT_DIR = $(BUILD_DIR)

# Module dependencies, variant settings, SDK defaults, the module description, and the SDK rules last.
include $(V_TARGET)_deps.min
include $(HEXAGON_SDK_ROOT)/build/make.d/$(V_TARGET)_vs.min
include $(HEXAGON_SDK_ROOT)/build/defines.min
include $(V_TARGET).min
include $(RULES_MIN)
