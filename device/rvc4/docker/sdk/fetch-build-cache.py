#!/usr/bin/env python3
"""Get the published build cache of the firmware build for the first of the given commits that has one.

usage: fetch-build-cache.py <base url> <build cache dir> <vcpkg binary cache dir> <commit>...

The commits are the commit of the checkout and its parents, newest first. A published build cache has two parts:
object files as a compiler cache (ccache), and the vcpkg dependencies as binary archives
(ci/device-rvc4-build-cache.sh makes them). The compiler cache goes to <build cache dir>/commits/<commit>/ccache, and
the archives go into the vcpkg binary cache. The script prints the commit that it took, or nothing when no commit
has a published cache.

The build works without a published cache, so this script does not stop it: an error gives a message and no cache.
"""
import hashlib
import json
import os
import re
import shutil
import sys
import tarfile
import tempfile
import time
import urllib.error
import urllib.request

FORMAT = 1
ASK_AGAIN = 24 * 3600  # seconds until the server is asked again for a commit that had no published cache
KEEP = 3  # the compiler caches of this number of commits stay on the disk


def log(text):
    print(text, file=sys.stderr, flush=True)


def download(url, path, sha256):
    digest = hashlib.sha256()
    with urllib.request.urlopen(url, timeout=60) as response, open(path, "wb") as out:
        for block in iter(lambda: response.read(1 << 20), b""):
            digest.update(block)
            out.write(block)
    if digest.hexdigest() != sha256:
        raise ValueError(f"the sha256 of {url} is not {sha256}")


def extract(archive, destination):
    """Extract an archive of regular files and folders. A member with another type or a path outside stops it."""
    with tarfile.open(archive) as tar:
        for member in tar:
            name = os.path.normpath(member.name)
            if os.path.isabs(name) or name.split(os.sep)[0] == "..":
                raise ValueError(f"member outside of the archive folder: {member.name}")
            target = os.path.join(destination, name)
            if member.isdir():
                os.makedirs(target, exist_ok=True)
            elif member.isfile():
                os.makedirs(os.path.dirname(target), exist_ok=True)
                with tar.extractfile(member) as source, tempfile.NamedTemporaryFile(dir=os.path.dirname(target), delete=False) as out:
                    shutil.copyfileobj(source, out)
                os.chmod(out.name, 0o644)
                os.replace(out.name, target)  # Atomic rename: another build can read the vcpkg cache at the same time
            else:
                raise ValueError(f"member that is not a file or a folder: {member.name}")


def part(manifest, name):
    entry = manifest[name]
    if not re.fullmatch(r"[0-9a-f]{64}", entry["sha256"]) or ".." in entry["path"] or entry["path"].startswith("/"):
        raise ValueError(f"the description of the published build cache has a wrong {name} entry")
    return entry


def install_vcpkg(base, root, vcpkg_cache, manifest):
    entry = part(manifest, "vcpkg")
    if not re.fullmatch(r"[0-9a-f]{64}", entry["key"]):
        raise ValueError("the description of the published build cache has a wrong vcpkg key")
    marker = os.path.join(root, "vcpkg", entry["key"])
    if os.path.exists(marker):
        return
    log(f"Published build cache: downloading the prebuilt vcpkg dependencies ({entry['size'] / 1e6:.0f} MB) ...")
    with tempfile.TemporaryDirectory(dir=root) as tmp:
        archive = os.path.join(tmp, "vcpkg.tar.xz")
        download(f"{base}/{entry['path']}", archive, entry["sha256"])
        extract(archive, vcpkg_cache)
    os.makedirs(os.path.dirname(marker), exist_ok=True)
    open(marker, "w").close()


def install_ccache(base, root, commit, manifest, text):
    entry = part(manifest, "ccache")
    log(f"Published build cache: downloading the compiler cache of commit {commit[:12]} ({entry['size'] / 1e6:.0f} MB) ...")
    tmp = tempfile.mkdtemp(dir=root)
    try:
        archive = os.path.join(tmp, "ccache.tar.xz")
        download(f"{base}/{entry['path']}", archive, entry["sha256"])
        extract(archive, os.path.join(tmp, "ccache"))
        os.remove(archive)
        with open(os.path.join(tmp, "build-cache.json"), "w") as out:
            out.write(text)
        os.chmod(tmp, 0o755)
        try:
            os.rename(tmp, os.path.join(root, "commits", commit))
        except OSError:
            pass  # another build put it there at the same time
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def take(base, root, vcpkg_cache, commit):
    """True when the published build cache of the commit is on the disk after the call."""
    local = os.path.join(root, "commits", commit)
    description = os.path.join(local, "build-cache.json")
    missing = os.path.join(root, "missing", commit)
    if not os.path.isfile(description):
        if os.path.exists(missing) and time.time() - os.path.getmtime(missing) < ASK_AGAIN:
            return False
        try:
            with urllib.request.urlopen(f"{base}/commits/{commit}/build-cache.json", timeout=15) as response:
                text = response.read().decode()
        except urllib.error.HTTPError as error:
            if error.code != 404:
                raise
            os.makedirs(os.path.dirname(missing), exist_ok=True)
            open(missing, "w").close()
            return False
        manifest = json.loads(text)
        if manifest.get("format") != FORMAT or manifest.get("commit") != commit:
            raise ValueError(f"the published build cache of {commit[:12]} has a format that this script does not know")
        install_ccache(base, root, commit, manifest, text)
    try:  # ccache is useful without prebuilt deps: vcpkg then builds them from source
        with open(description) as source:
            install_vcpkg(base, root, vcpkg_cache, json.load(source))
    except (urllib.error.URLError, OSError, ValueError, KeyError, TypeError, tarfile.TarError) as error:
        log(f"Published build cache: no prebuilt vcpkg dependencies ({error})")
    os.utime(local)
    return True


def main():
    if len(sys.argv) < 5:
        sys.exit(__doc__)
    base, root, vcpkg_cache, commits = sys.argv[1].rstrip("/"), sys.argv[2], sys.argv[3], sys.argv[4:]
    os.makedirs(os.path.join(root, "commits"), exist_ok=True)
    try:
        for commit in commits:
            if not re.fullmatch(r"[0-9a-f]{40}", commit):
                continue
            if take(base, root, vcpkg_cache, commit):
                print(commit)
                break
    except (urllib.error.URLError, OSError, ValueError, KeyError, TypeError, tarfile.TarError) as error:
        log(f"Published build cache: not used ({error})")
    try:
        folders = [os.path.join(root, "commits", name) for name in os.listdir(os.path.join(root, "commits"))]
        for folder in sorted(folders, key=os.path.getmtime, reverse=True)[KEEP:]:
            shutil.rmtree(folder, ignore_errors=True)
    except OSError:
        pass


if __name__ == "__main__":
    main()
