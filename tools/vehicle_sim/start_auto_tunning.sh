#!/bin/bash
pwd=$(pwd -P)
lwd=$(dirname $pwd)
time=$(date "+%Y%m%d%H%M%S")


target_file=${pwd}/config/simulator_config.yaml
target_field_1=lon_params_estimator_type
target_params_list_1=('RLS' 'RLS2' 'RLS3')
target_field_2=rls_type
target_params_list_2=('RLS' 'RAWRLS' 'RGTLS')
target_field_3=const_ref_v
target_params_list_3=(true false)

filename=${target_file%.*}
postfix=${target_file##*.}

cp ${filename}.${postfix} ${filename}_copy.${postfix}


for param_1 in ${target_params_list_1[@]}
do
    substitutive_cmd_1=s/${target_field_1}:\s*.*,/${target_field_1}": "${param_1},/g
    sed -i "$substitutive_cmd_1" ${target_file}
    for param_2 in ${target_params_list_2[@]}
    do
        substitutive_cmd_2=s/${target_field_2}:\s*.*,/${target_field_2}": "${param_2},/g
        sed -i "$substitutive_cmd_2" ${target_file}
        for param_3 in ${target_params_list_3[@]}
        do
            substitutive_cmd_3=s/${target_field_3}:\s*.*,/${target_field_3}": "${param_3},/g
            sed -i "$substitutive_cmd_3" ${target_file}
            echo ${target_field_1}' = '${param_1}' '${target_field_2}' = '${param_2}' '${target_field_3}' = '${param_3}
            python vehicle_simulator.py --output-dir ${pwd}/output/${time}
            sleep 5
        done
    done

done

mv ${filename}_copy.${postfix} ${filename}.${postfix}
