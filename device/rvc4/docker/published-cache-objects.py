#!/usr/bin/env python3
"""Which object files of the firmware build can go into the published build cache.

An object file has code of the headers that its source includes. Luxonis may not redistribute the Qualcomm SDKs of
this build (the Qualcomm AI Runtime SDK, FastCV) in any form, so an object file can be published only when its
source includes none of their headers. docker/build.sh uses this script when a CI job makes the published build
cache (ci/device-rvc4-build-cache.sh).

usage: published-cache-objects.py list <build dir> <config> <jobs>
           Print the object files (ninja targets) that are not built and that can be published. The script asks the
           compiler for the headers of each source (-M: preprocess only). A source that does not preprocess is not
           printed.
       published-cache-objects.py check <build dir> <config>
           Read the headers that the compiler used for each object file that is built (ninja -t deps). Exit status 1
           and the list of the object files when one of them used a header of a Qualcomm SDK.
"""
import concurrent.futures
import json
import os
import re
import shlex
import subprocess
import sys

# A header of a Qualcomm SDK, by its path. The words also match files of this repository that wrap these SDKs: a source
# that includes such a file is not published either.
QUALCOMM_HEADER = re.compile(r"qairt|aistack|snpe|fastcv|/opt/qcom/", re.IGNORECASE)


def parse(entry):
    """One compile command as (object file, directory, command that prints the headers of the source)."""
    words = shlex.split(entry["command"])
    if os.path.basename(words[0]) == "ccache":
        words = words[1:]
    command, output, option = [], None, None
    for word in words:
        if option == "-o":
            output = word
        if option:
            option = None
        elif word in ("-o", "-MT", "-MQ", "-MF"):
            option = word
        elif word not in ("-c", "-MD", "-MMD"):
            command.append(word)
    return output, entry["directory"], command + ["-M"]


def headers_of(compile_command):
    """The text of the dependency rule of one source, or None when the source does not preprocess."""
    _, directory, command = compile_command
    result = subprocess.run(command, cwd=directory, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    return result.stdout if result.returncode == 0 else None


def list_objects(build, config, jobs):
    with open(os.path.join(build, "compile_commands.json")) as source:
        commands = [parse(entry) for entry in json.load(source)]
    # The compile commands of the config, for the object files that are not built
    commands = [c for c in commands if c[0] and f"/{config}/" in c[0] and not os.path.exists(os.path.join(c[1], c[0]))]
    with concurrent.futures.ThreadPoolExecutor(jobs) as pool:
        for (output, directory, _), headers in zip(commands, pool.map(headers_of, commands)):
            if headers is not None and not QUALCOMM_HEADER.search(headers):
                print(os.path.relpath(os.path.join(directory, output), build))


def check(build, config):
    deps = subprocess.run(["ninja", "-C", build, "-f", f"build-{config}.ninja", "-t", "deps"], stdout=subprocess.PIPE, text=True, check=True).stdout
    bad, count, target = {}, 0, None
    for line in deps.splitlines():
        if line and not line.startswith(" "):
            target = line.split(":")[0]
            count += 1
        elif target and QUALCOMM_HEADER.search(line):
            bad.setdefault(target, line.strip())
    for name, header in sorted(bad.items()):
        print(f"{name}: {header}", file=sys.stderr)
    if bad or count == 0:
        sys.exit(f"error: {len(bad)} of {count} object files for the published build cache used a header of a Qualcomm SDK")
    print(f"Published build cache: {count} object files, none of them used a header of a Qualcomm SDK")


if __name__ == "__main__":
    if len(sys.argv) == 5 and sys.argv[1] == "list":
        list_objects(sys.argv[2], sys.argv[3], int(sys.argv[4]))
    elif len(sys.argv) == 4 and sys.argv[1] == "check":
        check(sys.argv[2], sys.argv[3])
    else:
        sys.exit(__doc__)
