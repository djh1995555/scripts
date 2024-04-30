#!/bin/bash
pwd=$(pwd -P)
lwd=$(dirname $pwd)
    
target_project=/home/mi/tmp_repo_mipilot_mbf_1111_lo5nm2

default_target_record_dir=$pwd/record/test_record/0418/0060/1659
target_record_dir=${1:-$default_target_record_dir}

time=$(date "+%Y%m%d%H%M%S")
parent_dir=$(dirname $target_record_dir)
output_dir=${pwd}/record/replay_record/$time/$(basename $parent_dir)/$(basename $target_record_dir)

output_filename=output.txt
python ${pwd}/record_replayer/record_replayer.py --target-record-dir $target_record_dir --output-dir $output_dir --project-dir $target_project 
mv $output_filename $output_dir
bash ${pwd}/start_generate_report.sh $output_dir
