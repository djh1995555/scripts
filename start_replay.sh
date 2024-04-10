#!/bin/bash
pwd=$(pwd -P)
lwd=$(dirname $pwd)

target_project=/home/mi/tmp_repo_mipilot_mbf_debug_9645_tzfrni
# default_target_record=$pwd/record/issue_record/0408/88048/10987299_EVENT_KEY_TRIGGER_APA_PARKING_EXCEPTION.record
default_target_record=$pwd/record/issue_record/0408/84115/7989494_EVENT_KEY_TRIGGER_APA_PARKING_EXCEPTION.record
target_record=${1:-$default_target_record}

time=$(date "+%Y%m%d%H%M%S")
target_record_dir=$(dirname $target_record)
parent_dir=$(dirname $target_record_dir)
output_dir=${pwd}/record/replay_record/$time/$(basename $parent_dir)/$(basename $target_record_dir)

python ./record_replayer.py --target-record $target_record --output-dir $output_dir --project-dir $target_project
bash ./start_generate_report.sh $output_dir
