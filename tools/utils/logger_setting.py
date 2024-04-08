#!/usr/bin/env python
import sys
import os
import logging

# default loglevel to INFO, loglevel could be changed by setting envrionment variable LOGLEVEL,
# available levels are DEBUG, INFO, WARNING, ERROR, CRITICAL
def set_logger(logger, log_dir, log_filename, log_level, console_level):
    log_formatter = logging.Formatter("[%(asctime).19s %(levelname)s %(filename)s:"
                                      "%(lineno)s]: %(message)s ")
    # add log console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(log_formatter)
    console_handler.setLevel(console_level)
    logger.addHandler(console_handler)
    
    # add log file handler
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    file_handler = logging.FileHandler(os.path.join(log_dir, log_filename), mode='w')
    file_handler.setFormatter(log_formatter)
    file_handler.setLevel(log_level)
    logger.addHandler(file_handler)
    