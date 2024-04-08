#!/bin/bash
mode=${1:-train}

config_name=silverbox_config.yaml

if [ ${mode} == 'train' ];then
    python ./control_learning/src/time_series_network.py  --config-filepath ${config_name} 
elif [ ${mode} == 'only_test' ];then
    python ./control_learning/src/time_series_network.py --config-filepath ${config_name} --only-test True
else
	echo "[ERROR]: mode must be 'train', 'only_test'"
fi
 