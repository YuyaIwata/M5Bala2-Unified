#!/usr/bin/env python3
"""Regenerate .vscode/c_cpp_properties.json from the real build.

Run `arduino-cli compile --profile m5stack_fire --build-path build .` first so
that build/compile_commands.json is current, then run this script from the
repository root.

ESP32 core 3.x passes most flags through GCC response files (@file) together
with -iprefix / -iwithprefixbefore. The C/C++ extension expands neither, so the
paths are resolved here and written out as plain absolute paths.
"""

import json
import os
import shlex
import sys

HOME = os.path.expanduser("~")
BUILD_DB = "build/compile_commands.json"
CONFIG = ".vscode/c_cpp_properties.json"


def expand(args):
    """Expand @response files recursively, keeping argument order."""
    out = []
    for arg in args:
        if arg.startswith("@"):
            path = arg[1:]
            if not os.path.exists(path):
                continue
            with open(path) as f:
                out.extend(expand(shlex.split(f.read())))
        else:
            out.append(arg)
    return out


def collect(args):
    """Return (include_paths, defines, std) from a fully expanded command."""
    includes, defines, std = [], [], None
    prefix = ""
    i = 0
    while i < len(args):
        a = args[i]
        if a == "-iprefix":
            prefix = args[i + 1]
            i += 2
            continue
        if a == "-iwithprefixbefore":
            includes.append(os.path.normpath(prefix + args[i + 1]))
            i += 2
            continue
        if a in ("-I", "-isystem"):
            includes.append(args[i + 1])
            i += 2
            continue
        if a.startswith("-I"):
            includes.append(a[2:])
        elif a.startswith("-D"):
            defines.append(a[2:])
        elif a.startswith("-std="):
            std = a[5:]
        i += 1
    return includes, defines, std


def portable(path):
    return path.replace(HOME, "${env:HOME}", 1) if path.startswith(HOME) else path


def main():
    if not os.path.exists(BUILD_DB):
        sys.exit(f"{BUILD_DB} not found. Build the sketch with --build-path build first.")

    db = json.load(open(BUILD_DB))
    entry = next((e for e in db if e["file"].endswith(".ino.cpp")), None)
    if entry is None:
        sys.exit("No sketch entry found in the compilation database.")

    args = expand(entry["arguments"])
    includes, defines, std = collect(args)

    # Preserve order while dropping duplicates and paths that no longer exist.
    seen, paths = set(), []
    for p in includes:
        if p not in seen and os.path.isdir(p):
            seen.add(p)
            paths.append(portable(p))

    cfg = json.load(open(CONFIG))
    c = cfg["configurations"][0]
    c["compilerPath"] = portable(args[0])
    c["includePath"] = ["${workspaceFolder}"] + paths
    c["defines"] = defines
    if std:
        c["cppStandard"] = std

    with open(CONFIG, "w") as f:
        json.dump(cfg, f, indent=4, ensure_ascii=False)
        f.write("\n")

    print(f"includePath: {len(c['includePath'])}, defines: {len(defines)}, cppStandard: {c['cppStandard']}")


if __name__ == "__main__":
    main()
