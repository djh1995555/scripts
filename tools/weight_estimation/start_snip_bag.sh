#!/bin/bash
pwd=$(pwd -P)
lwd=$(dirname $pwd)

start=0
end=300
output_dir=/mnt/intel/jupyterhub/jianhao.dong/snipped_bag
python ./snip_bag.py \
    --bag-list-file ${pwd}/result/days_with_max_error_of_mean_value.csv \
    --output-dir ${output_dir} \
    --start ${start} \
    --end ${end}