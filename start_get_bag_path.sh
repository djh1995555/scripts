#!/bin/bash
pwd=$(pwd -P)
lwd=$(dirname $pwd)
time=$(date "+%Y%m%d%H%M%S")

# get bag path, the time interval and vehicle are set in scripts/tools/sql_tools/sql_config.yaml
python ./tools/sql_tools/get_bagpath.py
