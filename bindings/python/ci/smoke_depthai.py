import os, sys, traceback, platform

try:
    import depthai as dai
except Exception:
    traceback.print_exc()
    sys.exit(1)

expected = os.environ.get("EXPECTED_VERSION") or os.environ.get("ver")
installed = getattr(dai, "__version__", "<unknown>")
if installed != expected:
    print(f"Version mismatch: installed {installed} vs expected {expected}", file=sys.stderr)
    sys.exit(1)

print("depthai:", installed)
print("python:", platform.python_version(), "ABI:", sys.implementation.cache_tag)