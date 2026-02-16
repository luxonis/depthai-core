import sys
import types

from depthai._C import *  # noqa: F401, F403
from depthai import _C  # noqa: F401

# Wildcard import doesn't cover dunder attributes; re-export them explicitly
from depthai._C import (  # noqa: F401
    __version__,
    __commit__,
    __commit_datetime__,
    __build_datetime__,
    __device_version__,
    __bootloader_version__,
    __device_rvc3_version__,
    __device_rvc4_version__,
)

# Register C++ submodules in sys.modules so that
# `import depthai.node`, `import depthai.log`, etc. still work
for _name in dir(_C):
    _attr = getattr(_C, _name)
    if isinstance(_attr, types.ModuleType):
        sys.modules[f"depthai.{_name}"] = _attr
