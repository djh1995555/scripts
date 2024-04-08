#!/bin/bash
pwd=$(pwd -P)
sudo rm -f /dev/shm/*
time=$(date "+%Y%m%d%H%M%S")
output_dir=${pwd}/report/runtime_report/custom_report
vehicle_name=48985

python ./tools/runtime_plotter.py --time ${time} --output-dir ${output_dir} --vehicle-name ${vehicle_name}  --enable-output True

bag_list=${output_dir}/${time}
python ./tools/report_generator.py ${bag_list} --output-dir ${output_dir}/${time}

 