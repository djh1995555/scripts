#!/bin/bash
mode=${1:-train}
task=${2:-weight}
config_name=weight_estimation_config.yaml

if [ ${task} == 'weight' ];then
    config_name=weight_estimation_config.yaml
elif [ ${task} == 'dynamic_model' ];then
    config_name=dynamic_model_config.yaml
elif [ ${task} == 'silver_box' ];then
    config_name=silver_box_config.yaml
elif [ ${task} == 'simple_system' ];then
    config_name=simple_system_config.yaml
else
	echo "[ERROR]: task must be 'weight', 'dynamic_model' or 'silver_box'"
fi

if [ ${mode} == 'train' ];then
    python ./control_learning/src/time_series_network.py  --config-filepath ${config_name} --task train
elif [ ${mode} == 'test' ];then
    python ./control_learning/src/time_series_network.py --config-filepath ${config_name} --task test
elif [ ${mode} == 'validation' ];then
    python ./control_learning/src/time_series_network.py --config-filepath ${config_name} --task validation
else
	echo "[ERROR]: mode must be 'train', 'only_test'"
fi
 