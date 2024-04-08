#!/bin/bash
mode=${1:-sim}
pwd=$(pwd -P)
lwd=$(dirname $pwd)
time=$(date "+%Y%m%d%H%M%S")

# target_file=${lwd}/drive/control_v2/config/j7/trucksim_config.prototxt
# target_field=total_weight
# target_params_list=(34000 37000 40000 43000 46000)
# target_vehicle_list=(j7 pdb)

# target_file=${lwd}/simulator/scenario/config/trucksim_config.prototxt.j7
# target_field=total_weight
# target_params_list=(30000 34000 37000 40000 43000)
# target_vehicle_list=(pdb)

target_file=${lwd}/drive/control_v2/config/j7-l4e/controller_param.prototxt
target_field=forget_factor
target_params_list=(0.99 0.999 0.9999)
target_vehicle_list=(pdb)

filename=${target_file%.*}
postfix=${target_file##*.}

cp ${filename}.${postfix} ${filename}_copy.${postfix}

for param in ${target_params_list[@]}
do
    for vehicle in ${target_vehicle_list[@]}
    do
        substitutive_cmd=s/${target_field}:\s*.*/${target_field}:${param}/
        sed -i ${substitutive_cmd} ${target_file}
        echo 'current param = '${param}
        if [ ${mode} == 'sim' ];then
            ./start_simulation.sh single navi_map ${vehicle} ${target_field}_${param} ${time}
        elif [ ${mode} == 'replay' ];then
            ./start_node_replayer.sh batch_list control ${target_field}_${param} ${time}
        fi
        sleep 5
    done
done	


mv ${filename}_copy.${postfix} ${filename}.${postfix}
