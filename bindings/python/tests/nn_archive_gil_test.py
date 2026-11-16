"""Device-free regression for Python thread progress during native archive I/O."""

import os
import subprocess
import sys

import pytest


@pytest.mark.skipif(not hasattr(os, "mkfifo"), reason="Requires POSIX named pipes")
@pytest.mark.parametrize("overload", ["default", "compression", "options"])
def test_archive_io_releases_gil(tmp_path, overload):
    """Let a Python writer unblock each native constructor and preserve errors."""
    # A Python writer must run while the constructor waits for archive bytes.
    # With the GIL held, both threads deadlock. Isolate the call so a regression
    # fails with a bounded timeout rather than hanging the pytest process.
    script = r'''
import os
import sys
import threading

import depthai as dai

path, overload = sys.argv[1:]
os.mkfifo(path)

def write_invalid_archive():
    with open(path, "wb", buffering=0) as stream:
        stream.write(b"This is not an NN archive.\n")

writer = threading.Thread(target=write_invalid_archive, daemon=True)
writer.start()
try:
    if overload == "default":
        dai.NNArchive(path)
    elif overload == "compression":
        dai.NNArchive(path, dai.NNArchiveEntry.Compression.AUTO)
    else:
        dai.NNArchive(path, dai.NNArchiveOptions())
except RuntimeError:
    # A corrupt archive must still propagate its normal Python exception.
    pass
else:
    raise AssertionError("Invalid archive was accepted")
writer.join(timeout=1)
assert not writer.is_alive(), "Archive writer did not finish"
'''
    result = subprocess.run(
        [sys.executable, "-c", script, str(tmp_path / "archive.tar"), overload],
        capture_output=True,
        text=True,
        timeout=15,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
