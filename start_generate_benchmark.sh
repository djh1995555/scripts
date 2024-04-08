#!/bin/bash

scenario_dir=/home/jianhao_dong/bags/pluscene_protobin
base_file_name=${scenario_dir}/base.protobin.template
for file in $(ls ${scenario_dir})
do 
    postfix=${file##*.}
    file_name=${file%%.*}
    if [ ${postfix} != 'protobin' ];then
        continue
    fi
    target_file_name=${scenario_dir}/${file}.template
    cp ${base_file_name} ${target_file_name}
    substitutive_cmd=s/scenario_files*.*/scenario_file:\"${file}\"/
    # sed -i ${substitutive_cmd} ${target_file_name}
    # zip ${scenario_dir}/${file_name}.zip ${scenario_dir}/${file_name}*
done	

