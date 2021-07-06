#!/bin/env python3
"""
Scripts that takes a pkg-config library name and convert it to `#pragma cling` related code for loading the 3rd party
library in the C++ Jupyter Notebook

Usage:
generate_cling_3rd_party.py libname [--target-dir .]
"""
from os import makedirs
from os.path import join
from subprocess import run
from typing import List, Tuple
import argparse

def retrieve_pkg_config_flags(libname: str)->List[str]:
    """
    Run pkg-config in the terminal and retrieve result
    """
    completed_process = run(["pkg-config", "--cflags", "--libs", libname], capture_output=True, universal_newlines=True)
    if completed_process.returncode != 0:
        print("Error: failed to find", libname, "in pkg-config")
        from sys import exit
        exit(1)
    return completed_process.stdout.replace("\n", "").split(" ")

def split_flags(flags: List[str])->Tuple[List[str], List[str], List[str]]:
    """
    Split flags into include_paths, library_paths, or libraries
    """
    include_paths = list()
    library_paths = list()
    libraries = list()

    for flag in flags:
        if flag.startswith("-I"):
            include_paths.append(flag[2:])
        elif flag.startswith("-L"):
            library_paths.append(flag[2:])
        elif flag.startswith("-l"):
            libraries.append(flag[2:])
        else:
            libraries.append(flag)
    
    return include_paths, library_paths, libraries

def write_cling_pragmas_to_file(include_paths, library_paths, libraries, filename):
    with open(filename, "w") as f:
        f.write("#pragma once\n\n")
        for x in include_paths:
            # -I to add_include_path()
            f.write('#pragma cling add_include_path("%s")\n' % x)
        for x in library_paths:
            # -L to #pragma cling add_library_path()
            f.write('#pragma cling add_library_path("%s")\n' % x)
        for x in libraries:
            # -l & /path/to.so to #pragma cling load()
            f.write('#pragma cling load("%s")\n' % x)

def main():
    parser = argparse.ArgumentParser("generate_cling_3rd_party")
    parser.add_argument("libname", help="the name of the library in pkg-config", type=str)
    parser.add_argument("--target-dir", default=".", help="path to save the file", type=str)
    args = parser.parse_args()

    pkg_config_flags = retrieve_pkg_config_flags(args.libname)
    include_paths, library_paths, libraries = split_flags(pkg_config_flags)

    makedirs(args.target_dir, exist_ok=True)
    filename = join(args.target_dir, "load_%s.h"%args.libname)
    write_cling_pragmas_to_file(include_paths, library_paths, libraries, filename)
    print("Files written to", filename)

if __name__ == "__main__":
    main()
