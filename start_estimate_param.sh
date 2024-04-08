#!/bin/bash
pwd=$(pwd -P)
lwd=$(dirname $pwd)
mode=$1
sudo rm -f /dev/shm/*
time=$(date "+%Y%m%d%H%M%S")
output_dir=${pwd}/tools/param_fitting_output/1_param/2


if [ ${mode} == 'batch_list' ];then
    while read bag_fullpath
    do
        bag_list=${bag_list}${bag_fullpath},
    done < ./baglist.lst
    python ./tools/param_fitting_from_bag.py ${bag_list} --output-dir ${output_dir}
elif [ ${mode} == 'single' ];then
    bag_name=/home/jianhao.dong/bags/20230601T082341_pdb-l4e-b0003_20_0to50.db
    bag_name=/mnt/vault8/drives/2023-08-11/20230811T081124_pdb-l4e-b0003_9.db
    python ./tools/param_fitting_from_bag.py ${bag_name} --output-dir ${output_dir}
else
	echo "[ERROR]: mode must be 'batch' or 'single'"
fi