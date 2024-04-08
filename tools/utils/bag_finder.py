#!/usr/bin/env python
import os

class BagFinder:
    def __init__(self):
        self._baglist = []

    def append_filename_to_baglist(self, path, file_name):
        if (file_name.split('.'))[-1] == 'bag' or (file_name.split('.'))[-1] == 'db':
            self._baglist.append(os.path.abspath(os.path.join(path,file_name)))

    def find_bags_in_(self, dir_name_list):
        for dir_name in dir_name_list:
            if os.path.isfile(dir_name):
                self.append_filename_to_baglist('', os.path.abspath(dir_name))
            elif os.path.isdir(dir_name):
                for path, dirs, files in os.walk(dir_name):
                    for file_name in files:
                        self.append_filename_to_baglist(path,file_name)
        self._baglist = list(set(self._baglist))
        return self._baglist