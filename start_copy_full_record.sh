#!/bin/bash
pwd=$(pwd -P)
lwd=$(dirname $pwd)

url=msd@192.168.1.168
date='0412'
target_dir=@url:/data_mnt/prod/parking/0411

for item in "$target_dir"/*; do
    save_dir=$pwd/record/test_record/$date/$(basename $item)
    mkdir -p $save_dir
    cp $item/full_record* $save_dir
done


