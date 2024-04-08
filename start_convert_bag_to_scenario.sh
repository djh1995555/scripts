#!/bin/bash
pwd=$(pwd -P)
lwd=$(dirname $pwd)
time=$(date "+%Y%m%d%H%M%S")
mode=$1
type=$2

# for conversion from bag to scenario
# FLAGS_lead_info_topic:  /planning/lead_info
# FLAGS_localization_topic: /localization/state
# FLAGS_obstacle_topic: /perception/obstacles
# FLAGS_lane_detection_topic: /perception/lane_path
# FLAGS_vehicle_weight_topic: /vehicle/weight_report not exist
# FLAGS_vehicle_dbw_reports_topic: /vehicle/dbw_reports
# FLAGS_vehicle_topic: /vehicle/misc_1_report
# FLAGS_truck_state_topic: /vehicle/truck_state
# FLAGS_odom_topic: /navsat/odom
# FLAGS_plus_odom_topic: /plus/odom

# for report generation
# /planning/trajectory
# /vehicle/control_cmd
# /vehicle/status
# /imu/data


if [ ${mode} == 'single' ];then
    target_dir=${pwd}/scenarios/single_scenario
    target_dir_with_time=${target_dir}/${time}

    # bag_fullpath=${HOME}/bags/20230108T094908_pdb-l4e-b0004_15_0to30.db
    # bag_fullpath=/mnt/intel/jupyterhub/jianhao.dong/bags/20221205T095000_j7-l4e-LFWSRXSJ7M1F48985_2_1to101.bag
    # bag_fullpath=/mnt/intel/jupyterhub/jianhao.dong/bags/20221216T101606_pdb-l4e-b0005_1_552to1185.db
    # bag_fullpath=/home/jianhao_dong/bags/20230104T105054_pdb-l4e-b0003_1_130to250.db
    # bag_fullpath=/home/jianhao_dong/bags/20230108T094908_pdb-l4e-b0004_15_0to600.db
    # bag_fullpath=/home/jianhao_dong/bags/20230108T094908_pdb-l4e-b0004_14_1to1179.db
    bag_fullpath=/home/jianhao_dong/bags/20221117T134511_j7-l4e-LFWSRXSJ7M1F48985_4_1to601.db
    # bag_fullpath=/home/jianhao_dong/bags//fastbag/low_speed/20220806T093547_j7-l4e-LFWSRXSJXM1F50505_4_380to615.db
    # bag_fullpath=/home/jianhao_dong/bags/scenario2/20211223T102304_j7-l4e-c0005_LFWSRXSJ9M1F45294_1_1354to1374.db
    # bag_fullpath=/home/jianhao_dong/bags/20230104T105054_pdb-l4e-b0003_1_130to250.db
    
    # bag_fullpath=/home/jianhao_dong/bags/scenario/20220816T145904_j7-l4e-LFWSRXSJ5M1F50508_7_1130to1155.db
    # bag_fullpath=/home/jianhao_dong/bags/scenario/20230212T062356_pdb-l4e-b0004_10_1176to1201.db
    # bag_fullpath=/home/jianhao_dong/bags/20230114T082029_pdb-l4e-b0007_33.db
    bag_fullpath=/home/jianhao_dong/bags/scenario/20230212T062356_pdb-l4e-b0004_10_1176to1201.db
    # bag_fullpath=/home/jianhao_dong/bags/20221117T134511_j7-l4e-LFWSRXSJ7M1F48985_4_1to601.db
    bag_fullname=`basename ${bag_fullpath}`
    bag_name=${bag_fullname%%.*}
    output_file_path=${target_dir_with_time}/${bag_name}
    if [ ! -d ${output_file_path} ]; then
        mkdir -p ${output_file_path}
    fi

    if [ ${type} == 'navi_lane' ];then
        ${lwd}/simulator/scenario/build/relwithdebinfo/bin/convert_bag_to_simulator_scenario \
            --bag_files_path ${bag_fullpath} \
            --output_path ${output_file_path}/${bag_name}.protobin \
            --collect_lane_info true \
            --build_from_detection true \
            --l_filter_dis 20 \
            --localization_mapping_file ${output_file_path}/${bag_name}_mapping.prototxt

    elif [ ${type} == 'navi_map' ];then
        ${lwd}/simulator/scenario/build/relwithdebinfo/bin/convert_bag_to_simulator_scenario \
            --bag_files_path ${bag_fullpath} \
            --output_path ${output_file_path}/${bag_name}.protobin \
            --projection_coordinate eqdc \
            --l_filter_dis 30 \
            --plusmap_file ${pwd}/map/suzhou_circle_ccw_hd.undeflected.lane_geometry.pb \
            --plusmap_route_file ${pwd}/map/jiangsu_highway_and_local-20191217.plan_route.pb \
            --generate_scenario_in_global_frame
    else
        echo "input 'navi_lane' or 'navi_map'"
    fi
    echo "==============finish ${bag_name}==================="

    latest_scenario_filepath=${target_dir}/latest_scenario
    if [ -d ${latest_scenario_filepath} ]; then
        rm -r ${latest_scenario_filepath}	
    fi
    mkdir -p ${latest_scenario_filepath}

    for file in $(ls ${target_dir_with_time})
    do
        cp -r ${target_dir_with_time}/${file} ${latest_scenario_filepath}
    done

elif [ ${mode} == 'batch' ];then
    target_dir=${pwd}/scenarios/scenario_list
    target_dir_with_time=${target_dir}/${time}

    while read bag_fullpath
    do
        bag_fullname=`basename ${bag_fullpath}`
        bag_name=${bag_fullname%%.*}
        output_file_path=${target_dir_with_time}
        if [ ! -d ${output_file_path} ]; then
            mkdir -p ${output_file_path}
        fi
        ${lwd}/simulator/scenario/build/relwithdebinfo/bin/convert_bag_to_simulator_scenario \
            --bag_files_path ${bag_fullpath} \
            --output_path ${output_file_path}/${bag_name}.prototxt \
            --collect_lane_info true \
            --build_from_detection true \
            --l_filter_dis 10 \
            --localization_mapping_file ${output_file_path}/${bag_name}_mapping.prototxt
        echo "=======================convert ${bag_name} to scenario======================="
    done < ./baglist.lst


    latest_scenario_list=${target_dir}/latest_scenario_list
    if [ -d ${latest_scenario_list} ]; then
        rm -r ${latest_scenario_list}	
    fi
    mkdir -p ${latest_scenario_list}

    for file in $(ls ${target_dir_with_time})
    do
        cp -r ${target_dir_with_time}/${file} ${latest_scenario_list}
    done
fi



