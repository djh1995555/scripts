#!/bin/bash

# mode = single: replay one bag
# mode = batch: replay all bags in one folder
mode=$1
node=$2
pwd=$(pwd -P)
sudo rm -f /dev/shm/*
replay_dir=${pwd}/report/replay_node_output
# vehicle_name=j7-l4e-sim
vehicle_name=pdb-l4e-sim


time=$(date "+%Y%m%d%H%M%S")

if [ ${mode} == 'batch' ];then
    output_dir=${replay_dir}/${time}
    target_bag_path=${HOME}/bags/fastbag/low_speed
    for file in $(ls ${target_bag_path})
    do 
        postfix=${file##*.}
        if [ ${postfix} == 'db' ];then
            bag_list=${bag_list}${target_bag_path}/${file},
            # echo ${file}
        fi
    done
    python ./replay_node.py ${bag_list} --use-remap-topic true	  --output-dir ${output_dir} --target-node ${node} --vehicle-name ${vehicle_name}
elif [ ${mode} == 'batch_list' ];then
    while read bag_fullpath
    do
        bag_fullname=`basename ${bag_fullpath}`
        bag_name=${bag_fullname%%.*}
        output_dir=${replay_dir}/${time}/${bag_name}
        echo ${target_dir_name}
        if [ ! -d ${target_dir_name} ]; then
            mkdir -p ${target_dir_name}
        fi
        python ./replay_node.py ${bag_fullpath} --use-remap-topic true	--output-dir ${output_dir} --target-node ${node} --vehicle-name ${vehicle_name}
        
        python ./tools/report_generator.py ${output_dir} --output-dir ${output_dir}
    done < ${pwd}/baglist.lst
elif [ ${mode} == 'single' ];then
    output_dir=${replay_dir}/${time}
    bag_name=${HOME}/bags/20221128T124744_j7-l4e-LFWSRXSJ7M1F48985_0_460to560.db
    bag_name=${HOME}/bags/20221128T121641_j7-l4e-LFWSRXSJ7M1F48985_0_1088to1198.db
    bag_name=${HOME}/bags/20221205T095000_j7-l4e-LFWSRXSJ7M1F48985_2_1to201.db
    bag_name=${HOME}/bags/20220806T093547_j7-l4e-LFWSRXSJXM1F50505_4_380to615.db
    bag_name=${HOME}/bags/20220806T093547_j7-l4e-LFWSRXSJXM1F50505_5_653to803.db
    bag_name=${HOME}/bags/20221226T082755_pdb-l4e-b0008_1_1to31.bag
    bag_name=${HOME}/bags/20221223T093013_pdb-l4e-b0008_2_1to31.bag
    bag_name=${HOME}/bags/20221216T101606_pdb-l4e-b0005_1_1to32.bag
    bag_list=${bag_name}
    python ./replay_node.py ${bag_list} --use-remap-topic true	--output-dir ${output_dir} --target-node ${node} --vehicle-name ${vehicle_name}
    
    python ./tools/report_generator.py ${output_dir} --output-dir ${output_dir}
else
	echo "[ERROR]: mode must be 'batch' or 'single'"
fi

