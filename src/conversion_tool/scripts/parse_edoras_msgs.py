#!/usr/bin/env python3

import rclpy
import argparse
import os
import sys
import pathlib
from typing import List, Literal
from rosidl_adapter.parser import parse_message_file
from ament_index_python.packages import get_package_share_directory
from rosidl_adapter.resource import expand_template

def get_files(paths: str) -> List[str]:
    files: List[str] = []

    for path in paths:
        if os.path.isdir(path):
            for dirpath, dirnames, filenames in os.walk(path):
                dirnames[:] = [d for d in dirnames if d[0] not in ['.', '_']]
                dirnames.sort()

                # select files by extension
                for filename in sorted(filenames):
                    if filename.endswith('.msg'):
                        files.append(os.path.join(dirpath, filename))

        if os.path.isfile(path):
            files.append(path)
    return files

#########################
def main(args=None):
    rclpy.init(args=args)
 
    pkg_name = "edoras_msgs"
    paths = get_package_share_directory(pkg_name)
    print(paths)
    files = get_files([paths])

    for filename in files:
        #pkg_name = os.path.basename(os.path.dirname(os.path.dirname(filename)))
        try:
            msg_spec = parse_message_file(pkg_name, filename)
            print("** Message start...**")
            print(pkg_name, filename)
            basefile = pathlib.PurePath(os.path.basename(filename))
            output_file = pathlib.Path('/home/ana/Desktop') / basefile.with_suffix('.idl_test').name
            data = {
                'pkg_name': pkg_name,
                'relative_input_file': basefile.as_posix(),
                'msg': msg_spec
            }

            expand_template('msg.idl.em', data, output_file, encoding='iso-8859-1')
            print("Output file: ", output_file)
            print("** Message end! ....**")
        except Exception as e:
            print(' ', pkg_name, filename, str(e))
            raise


    
if __name__ == '__main__':
    main()
