#!/bin/bash
# mode = single: replay one bag
# mode = batch: replay all bags in one folder
# required topic
# control
# planning
# localization
# vehicle
# status_report
# other
# /imu/data
# /navsat/odom
# /novatel_data/inspva
# /novatel_data/inspvax

mode=$1
node=$2
param_tunning=${3:-default}
current_time=${4:-xxxx}
pwd=$(pwd -P)
lwd=$(dirname $pwd)
sudo rm -f /dev/shm/*
time=$(date "+%Y%m%d%H%M%S")
export GLOG_v=4
export GLOG_minloglevel=0
vehicle_name=pdb-l4e-sim
output_dir=${pwd}/report/replay_report

if [ ${current_time} == 'xxxx' ];then
	time=$(date "+%Y%m%d%H%M%S")	
else
	time=${current_time}
fi
if [ ${param_tunning} == 'default' ];then
	current_target_dir=${output_dir}/${time}	
else
	current_target_dir=${output_dir}/${time}/${param_tunning}
fi

if [ ! -d ${current_target_dir} ]; then
    mkdir -p ${current_target_dir}
fi

rosparam set use_sim_time true
if [ ${mode} == 'batch' ];then
    target_bag_path=${HOME}/bags/fastbag/low_speed
    for file in $(ls ${target_bag_path})
    do 
        postfix=${file##*.}
        if [ ${postfix} == 'db' ];then
            bag_list=${bag_list}${target_bag_path}/${file},
            # echo ${file}
        fi
    done
    python ./tools/node_replayer.py ${bag_list} \
        --drive-root ${lwd}/drive/opt/relwithdebinfo \
        --output-dir ${current_target_dir}\
        --vehicle-name ${vehicle_name} \
        --target-node ${node}

    python ./tools/report_generator.py ${current_target_dir} --output-dir ${current_target_dir}
elif [ ${mode} == 'batch_list' ];then
    vehicle_name=pdb-l4e-sim

    # while read bag_fullpath
    # do
    #     bag_list=${bag_list}${bag_fullpath},
    # done < ./baglist.lst

    # python ./tools/node_replayer.py ${bag_list} \
    #     --drive-root ${lwd}/drive/opt/relwithdebinfo \
    #     --output-dir ${current_target_dir}\
    #     --vehicle-name ${vehicle_name} \
    #     --target-node ${node} >& ${current_target_dir}/output.log

    # python ./tools/report_generator.py ${current_target_dir} --output-dir ${current_target_dir}

    while read bag_fullpath
    do
        echo "replay bag ${bag_fullpath}"
        bag_fullname=`basename ${bag_fullpath}`
        bag_name=${bag_fullname%%.*}
        sub_target_dir=${current_target_dir}/${bag_name}
        if [ ! -d ${sub_target_dir} ]; then
            mkdir -p ${sub_target_dir}
        fi
        python ./tools/node_replayer.py ${bag_fullpath} \
            --drive-root ${lwd}/drive/opt/relwithdebinfo \
            --output-dir ${sub_target_dir}\
            --vehicle-name ${vehicle_name} \
            --target-node ${node} >& ${sub_target_dir}/output.log

        python ./tools/report_generator.py ${sub_target_dir} --output-dir ${sub_target_dir}       
    done < ./baglist.lst

elif [ ${mode} == 'single' ];then
    # vehicle_name=pdc
    # bag_list=${HOME}/bags/20230214T134840_pdc-l4e-a0002_1_1to349.db

    vehicle_name=pdb-l4e-sim
    bag_list=${HOME}/bags/weight_estimation/20230104T105054_pdb-l4e-b0003_1_130to460.db # 43t
    
    bag_list=${HOME}/bags/weight_estimation/20230104T125119_pdb-l4e-b0005_2_749to949.db # 44.6t
    bag_list=${HOME}/bags/weight_estimation/20230412T060004_pdb-l4e-b0003_13_486to726.db # 36t
    bag_list=${HOME}/bags/weight_estimation/20230411T064756_pdb-l4e-b0008_3_760to940.db # 35.16t
    bag_list=${HOME}/bags/weight_estimation/20230323T045445_pdb-l4e-b0008_11_654to954.db # 29.78t

    bag_list=${HOME}/bags/weight_estimation/20230808T064649_pdb-l4e-b0008_3_514to694.db # 32.56t
    bag_list=${HOME}/bags/weight_estimation/20230802T064049_pdb-l4e-b0008_4_20to200.db  # 42.64t

    bag_list=${HOME}/bags/weight_estimation/20230412T060004_pdb-l4e-b0003_22_465to645.db # 36t
    bag_list=${HOME}/bags/weight_estimation/20230402T042858_pdb-l4e-b0008_10_1014to1254.db # 31.28t

    bag_list=${HOME}/bags/weight_estimation/20230702T060753_pdb-l4e-b0003_7_176to596.db  # 28.12t
    bag_list=${HOME}/bags/weight_estimation/20230702T060753_pdb-l4e-b0003_8_516to1038.db # 28.12t
    bag_list=${HOME}/bags/weight_estimation/20230702T070727_pdb-l4e-c0001_0_871to991.db # 28.3t
    bag_list=${HOME}/bags/weight_estimation/20230702T070727_pdb-l4e-c0001_1_372to552.db # 28.3t

    # bag_list=${HOME}/bags/weight_estimation/20230410T065944_j7-l4e-LFWSRXSJ4M1F50502_20_537to717.db # 30.7t
    # bag_list=${HOME}/bags/weight_estimation/20230331T235437_j7-l4e-LFWSRXSJ4M1F50502_5_1097to1307.db # 38.82t

    # bag_list=${HOME}/bags/20221117T134511_j7-l4e-LFWSRXSJ7M1F48985_4_1to601.db    # 36t 
    # bag_list=${HOME}/bags/20230103T024339_j7-l4e-LFWSRXSJ3M1F50507_5_407to767.db  # 30t
    
    # bag_list=/home/jianhao_dong/bags/fastbag/low_speed/20220806T093547_j7-l4e-LFWSRXSJXM1F50505_5_1403to1503.db
    # bag_list=/home/jianhao_dong/bags/fastbag/low_speed/20220803T100508_j7-l4e-LFWSRXSJ3M1F50507_2_1to21.db
    # bag_list=/home/jianhao_dong/bags/fastbag/low_speed/20220803T100508_j7-l4e-LFWSRXSJ3M1F50507_2_1to51.db
    # bag_list=/home/jianhao_dong/bags/fastbag/low_speed/20220804T070904_j7-l4e-LFWSRXSJ4M1F48989_13_173to198.db
    # bag_list=/home/jianhao_dong/bags/fastbag/low_speed/20220806T065103_j7-l4e-LFWSRXSJ8M1F50504_6_1095to1295.db
    # bag_list=/home/jianhao_dong/bags/fastbag/low_speed/20220806T065103_j7-l4e-LFWSRXSJ8M1F50504_6_1295to1445.db
    # bag_list=/home/jianhao_dong/bags/fastbag/low_speed/20220806T065103_j7-l4e-LFWSRXSJ8M1F50504_6_1445to1619.db
    # bag_list=/home/jianhao_dong/bags/fastbag/low_speed/20220806T093547_j7-l4e-LFWSRXSJXM1F50505_4_380to615.db
    # bag_list=/home/jianhao_dong/bags/fastbag/low_speed/20220806T093547_j7-l4e-LFWSRXSJXM1F50505_5_1103to1203.db
    # bag_list=/home/jianhao_dong/bags/fastbag/low_speed/20220806T093547_j7-l4e-LFWSRXSJXM1F50505_5_1203to1303.db
    # bag_list=/home/jianhao_dong/bags/fastbag/low_speed/20220806T093547_j7-l4e-LFWSRXSJXM1F50505_5_1303to1403.db
    # bag_list=/home/jianhao_dong/bags/fastbag/low_speed/20220806T093547_j7-l4e-LFWSRXSJXM1F50505_5_653to803.db

    
    python ./tools/node_replayer.py ${bag_list} \
        --drive-root ${lwd}/drive/opt/relwithdebinfo \
        --output-dir ${current_target_dir}\
        --vehicle-name ${vehicle_name} \
        --target-node ${node} >& ${current_target_dir}/output.log

    python ./tools/report_generator.py ${current_target_dir} --output-dir ${current_target_dir}
else
	echo "[ERROR]: mode must be 'batch' or 'single'"
fi