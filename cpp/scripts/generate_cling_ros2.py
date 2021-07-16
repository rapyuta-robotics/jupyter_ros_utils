#! /usr/bin/env python3
"""
Scripts that takes install/devel path of ROS2 workspace and convert it to `#pragma cling` related code for loading ROS2
in the C++ Jupyter Notebook

Usage
generate_cling_for_ros2.py /path/to/ros2_ws/install [--target-dir .]
"""
import argparse
from pathlib import Path
from typing import List, Tuple

def get_combo_flags(prefix_path: Path) -> Tuple[List[str], List[str], List[str]]:
    prefix_path = Path(prefix_path).expanduser().resolve()
    assert(prefix_path.exists())
    include_paths = [(prefix_path / 'include').as_posix()]
    library_paths = [(prefix_path / 'lib').as_posix()]
    libraries = list()
    for file in Path(library_paths[0]).iterdir():
        potential_lib = file.name.split('.so')
        if len(potential_lib) == 1:
            continue
        elif potential_lib[0] == "liblibstatistics_collector":
            continue
        libraries.append(potential_lib[0][3:])  # remove the lib prefix
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
    parser = argparse.ArgumentParser("generate_cling_prefix_path")
    parser.add_argument("prefix_path", help="location of ROS2 workspace (install/devel folder)", type=Path)
    parser.add_argument("--target-dir", default=".", help="path to save the file", type=Path)
    args = parser.parse_args()
    include_paths, library_paths, libraries = get_combo_flags(args.prefix_path)
    args.target_dir.mkdir(exist_ok=True)
    filename = args.target_dir / "load_ros2.h"
    write_cling_pragmas_to_file(include_paths, library_paths, libraries, filename)
    print("Files written to", filename)

if __name__ == "__main__":
    main()
