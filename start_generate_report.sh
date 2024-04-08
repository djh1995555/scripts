#!/bin/bash
pwd=$(pwd -P)
lwd=$(dirname $pwd)
mode=$1
sudo rm -f /dev/shm/*
time=$(date "+%Y%m%d%H%M%S")
output_dir=${pwd}/report/bag_report/custom_report


if [ ${mode} == 'batch_folder' ];then
    bag_folder=/mnt/intel/jupyterhub/jianhao.dong/bags/issue
    python ./tools/report_generator.py ${bag_folder} --output-dir ${output_dir}/${time}	
elif [ ${mode} == 'batch_list' ];then
    while read bag_fullpath
    do
        bag_list=${bag_list}${bag_fullpath},
    done < ./baglist.lst
    python ./tools/report_generator.py ${bag_list} --output-dir ${output_dir}/${time}	
elif [ ${mode} == 'single' ];then
    bag_name=/mnt/intel/jupyterhub/jianhao.dong/bags/issue/20221213T080500_pdb-l4e-b0008_28_245to265.bag
    bag_name=/home/jianhao_dong/l4e6/scripts/report/simulation_report/20230220132227/20230104T105054_pdb-l4e-b0003_1_130to250/simulation_output/bag/20230220T052233_pdb-l4e-sim_0.bag
    bag_name=/home/jianhao_dong/l4e6/scripts/report/simulation_report/20230221144141/scenario_coasting_02_05/simulation_output/bag/20230221T064147_j7-l4e-sim_0.bag
    bag_name=/home/jianhao_dong/bags/20221014T105131_j7-00010_2_65to125.db
    bag_name=/home/jianhao_dong/bags/20230103T024339_j7-l4e-LFWSRXSJ3M1F50507_5_407to767.db
    bag_name=/home/jianhao_dong/bags/20230104T105054_pdb-l4e-b0003_1_130to460.db
    bag_name=/home/jianhao_dong/bags/20230108T094908_pdb-l4e-b0004_15.db
    bag_name=/home/jianhao_dong/l4e6/scripts/report/benchmarks/20230302233151/benchmark_bag_cruise_speed/benchmarks/benchmark_merge_20220106T082027_j7-l4e-LFWSRXSJ0M1F48987_1_184to218/run_1/attempt_1/bag/20230302T224811_j7-l4e-sim_0.bag
    bag_name=/home/jianhao_dong/bags/20230307T123345_pdc-l4e-a0002_3_8to20.db
    bag_name=/home/jianhao_dong/bags/20230308T053553_pdb-l4e-sim_0.bag
    bag_name=/home/jianhao_dong/bags/20230308T134514_j7-l4e-sim_0.bag
    python ./tools/report_generator.py ${bag_name} --output-dir ${output_dir}/${time}	
else
	echo "[ERROR]: mode must be 'batch' or 'single'"
fi