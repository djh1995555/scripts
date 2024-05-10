#!/bin/bash
pwd=$(pwd -P)
lwd=$(dirname $pwd)
time=$(date "+%Y%m%d%H%M%S")
target_project=/home/mi/tmp_repo_mipilot_mbf_1111_lo5nm2
target_project=/home/mi/tmp_repo_mipilot_debug_41342_GRFRxH
# target_project=/home/mi/tmp_repo_mipilot_mbf_1096_fLUN9k
target_project=/home/mi/tmp_repo_mipilot_mbf_debug_11721_Aw3nfN

mode=$1

RunReplay(){
    target_record_dir=$1
    start_time=$2
    parent_dir=$(dirname $target_record_dir)
    date_dir=$(dirname $parent_dir)
    output_dir=${pwd}/record/replay_record/$time/$(basename $date_dir)/$(basename $parent_dir)/$(basename $target_record_dir)
    output_filename=output.txt
    python ${pwd}/record_replayer/record_replayer.py \
        --target-record-dir $target_record_dir \
        --output-dir $output_dir \
        --project-dir $target_project \
        --start-time $start_time \
        >& $output_filename
    mv $output_filename $output_dir
    bash ${pwd}/start_generate_report.sh $output_dir
    echo "***************************************************************"
}
    
if [ ${mode} == 'replay_single' ];then
    default_target_record_dir=$pwd/record/test_record/0418/0060/1659
    # default_target_record_dir=$pwd/record/test_record/2024-04-28/MT091/14-25-17 #88
    # default_target_record_dir=$pwd/record/test_record/2024-04-28/MT091/14-37-19 #42
    default_target_record_dir=$pwd/record/test_record/2024-04-28/MT091/15-59-51 #50
    # default_target_record_dir=$pwd/record/test_record/2024-04-28/MT091/16-26-10 #59
    # default_target_record_dir=$pwd/record/test_record/2024-04-28/MT091/16-44-03 #53
    # default_target_record_dir=$pwd/record/test_record/2024-04-28/MT091/16-49-24 #36,55
    # default_target_record_dir=$pwd/record/test_record/2024-04-28/MT091/16-51-08 #40
    # default_target_record_dir=$pwd/record/test_record/2024-04-28/MT091/16-52-53 #44
    # default_target_record_dir=$pwd/record/test_record/2024-04-28/MT091/16-55-22 #56
    # default_target_record_dir=$pwd/record/test_record/2024-04-28/MT091/16-59-51 #36
    # default_target_record_dir=$pwd/record/test_record/2024-04-28/MT091/20-32-41 #38
    # default_target_record_dir=$pwd/record/test_record/2024-04-28/MT091/20-34-39 #64
    # default_target_record_dir=$pwd/record/test_record/2024-04-28/MT091/20-36-59 #63


    # default_target_record_dir=$pwd/record/test_record/2024-04-18/MT091/10-32-59 # no
    # default_target_record_dir=$pwd/record/test_record/2024-04-18/MT091/10-35-14 # yes
    default_target_record_dir=$pwd/record/test_record/2024-04-18/MT091/10-48-56 # no
    # default_target_record_dir=$pwd/record/test_record/2024-04-18/MT091/10-53-04 # yes
    default_target_record_dir=$pwd/record/test_record/2024-04-18/MT091/10-55-15
    default_target_record_dir=$pwd/record/test_record/2024-04-21/MT091/11-32-11
    default_target_record_dir=$pwd/record/test_record/2024-05-09/2120
    

    target_record_dir=${2:-$default_target_record_dir}

    start_time="120"

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
fi
