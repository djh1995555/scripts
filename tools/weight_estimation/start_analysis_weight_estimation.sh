#!/bin/bash
pwd=$(pwd -P)
lwd=$(dirname $pwd)
mode=$1

select_num_per_day=10

num_per_vehicle=3
num_per_day=1
read_bag_length=10

if [ ${mode} == 'query' ];then
    python get_bagpath_for_weight_estimation.py --output-dir ${pwd}/bags_queried_for_weight_estimation --selected-num-per-day ${select_num_per_day}
elif [ ${mode} == 'read' ];then
    python read_weight_estimation.py \
           --read-bag-length ${read_bag_length}\
           --num-per-vehicle ${num_per_vehicle}\
           --num-per-day ${num_per_day}
elif [ ${mode} == 'report' ];then
    result_filepath="/home/jianhao.dong/l4e2/scripts/tools/weight_estimation/analysis_output/2023-12-11_10-55-32-244284/weight_estimation_analysis_result.csv"
    python read_weight_estimation.py \
           --result-existed true\
           --result-filepath ${result_filepath}\
           --read-bag-length ${read_bag_length}\
           --num-per-vehicle ${num_per_vehicle}\
           --num-per-day ${num_per_day}
elif [ ${mode} == 'both' ];then
    python get_bagpath_for_weight_estimation.py --output-dir ${pwd}/bags_queried_for_weight_estimation --selected-num-per-day ${select_num_per_day}
    python read_weight_estimation.py \
           --read-bag-length ${read_bag_length}\
           --num-per-vehicle ${num_per_vehicle}\
           --num-per-day ${num_per_day}
else
    echo "[ERROR]: mode must be 'query', 'read' or both"
fi