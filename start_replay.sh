#!/bin/bash
pwd=$(pwd -P)
lwd=$(dirname $pwd)
time=$(date "+%Y%m%d%H%M%S")

mode=$1
default_target_record_dir=$pwd/record/test_record/2024-05-09/2120
target_record_dir=${2:-$default_target_record_dir}
default_target_project=/home/mi/ws/tmp_repo_mipilot_mbf_debug_v2_1162553_GiPyGV
target_project=${3:-$default_target_project}

RunReplay(){
    target_record_dir=$1
    start_time=$2
    parent_dir=$(dirname $target_record_dir)
    date_dir=$(dirname $parent_dir)
    output_dir=${pwd}/record/replay_record/$time/$(basename $date_dir)/$(basename $parent_dir)/$(basename $target_record_dir)
    output_filename=output.txt
    found=0

    target_record=$target_record_dir

    for item in "$target_record_dir"/*; do
        filename=$(basename $item)
        if [[ "$filename" == *.record || "$filename" == full_record.* ]]; then
            target_record=$item
            found=1
            break
        fi
    done
    if [ $found -eq 0 ];then
        echo "Error: No record file found in $target_record_dir"
        exit 1
    fi
    echo target_record:$target_record
    echo target_record_dir:$target_record_dir
    echo output_dir:$output_dir
    echo target_project:$target_project
    ./mcap_parse.sh $target_record $pwd
    echo "djh159djh" | sudo -S cp $pwd/system.json /dat/prod/device_hub/
    # sudo -S cp $pwd/system.json /dat/prod/device_hub/

    python ${pwd}/record_replayer/record_replayer.py \
        --target-record-dir $target_record_dir \
        --output-dir $output_dir \
        --project-dir $target_project \
        --start-time $start_time \
        >& $output_filename
    mv $output_filename $output_dir
    bash ${pwd}/start_generate_report.sh $output_dir
    mv $pwd/system.json $output_dir
    echo "***************************************************************"
}
    
if [ ${mode} == 'replay_single' ];then
 

    start_time="00"

    RunReplay $target_record_dir $start_time


elif [ ${mode} == 'replay_dir' ];then
    record_dir=$pwd/record/test_record/issue_record
    declare -a replay_target_dir

    function traverse_directory() {
        local dir=$1
        local has_record_file=false

        for item in "$dir"/*; do
            filename=$(basename $item)
            if [ -d "$item" ]; then
                traverse_directory "$item"
            elif [[ "$filename" == *.record || "$filename" == full_record.* ]]; then
                replay_target_dir+=("$dir")
                has_record_file=true
                break
            fi
        done

        if [ "$has_record_file" = false ]; then
            return
        fi
    }

    traverse_directory $record_dir

    for dir in "${replay_target_dir[@]}"; do
        echo replay target dir is $dir
        start_time="00"
        RunReplay $dir $start_time
    done | sort -u
    
elif [ ${mode} == 'replay_list' ];then
    declare -a replay_target_dir
    while read record_dir
    do
        # echo "replay record" ${record_dir}
        replay_target_dir+=("$record_dir")
    done < ./record_dirs.lst

    for dir in "${replay_target_dir[@]}"; do
        echo replay target dir is $dir
        start_time="00"
        RunReplay $dir $start_time
    done | sort -u
else
    echo "args wrong!"
fi
