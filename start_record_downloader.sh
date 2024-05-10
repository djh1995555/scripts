#!/bin/bash
pwd=$(pwd -P)
lwd=$(dirname $pwd)
    
python ${pwd}/record_downloader/record_downloader.py
