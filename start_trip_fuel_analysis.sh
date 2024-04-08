#!/bin/bash
mode=$1
type=$2
# config=J7_C_SAMPLE.yaml
config=K7_X13.yaml
pwd=$(pwd -P)
lwd=$(dirname $pwd)
time=$(date "+%Y%m%d%H%M%S")

cd ${lwd}/tools/fuel_economy/engine_operation_points_analysis

if [ ${mode} == 'trip' ];then
    target_dir_name=${pwd}/report/trip_fuel_report
    echo ${target_dir_name}
    if [ ! -d ${target_dir_name} ]; then
        mkdir -p ${target_dir_name}
    fi
    python trip_detail_analysis.py example/trip_info_config.yaml ${target_dir_name} --task 1 --truck_config_file ../common/truck_configs/${config}
elif [ ${mode} == 'bag' ];then
    target_dir_name=${pwd}/report/fe_report_bag
    echo ${target_dir_name}
    if [ ! -d ${target_dir_name} ]; then
        mkdir -p ${target_dir_name}
    fi
    python trip_detail_analysis.py example/trip_info_config.yaml ${target_dir_name} \
    --task 1 \
    --truck_config_file ../common/truck_configs/${config} \
    --data_collection_method bag
fi


