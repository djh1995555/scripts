#!/bin/bash
pwd=$(pwd -P)
lwd=$(dirname $pwd)

default_record_dir=$pwd/record/issue_record/0408
default_record_dir=/home/mi/debug/scripts/record/replay_record
# default_record_dir=/home/mi/debug/scripts/record/issue_record/0408/84115
default_record_dir=/home/mi/debug/scripts/record/replay_record/20240410171449/0408/88048
echo $default_record_dir

record_dir=${1:-$default_record_dir}
time=$(date "+%Y%m%d%H%M%S")
output_dir=${pwd}/report/issue_report

if [ ! -d $record_dir ]; then
    echo "Error: $record_dir is not a directory"
    exit 1
fi

declare -a result_dirs

function traverse_directory() {
    local dir=$1
    local has_record_file=false

    for item in "$dir"/*; do
        filename=$(basename $item)
        if [ -d "$item" ]; then
            traverse_directory "$item"
        elif [[ "$filename" == *.record || "$filename" == full_record.* ]]; then

            result_dirs+=("$dir")
            has_record_file=true
            break
        fi
    done

    if [ "$has_record_file" = false ]; then
        return
    fi
}

traverse_directory $record_dir


for dir in "${result_dirs[@]}"; do
    $lwd/mi_tools/bazel-bin/mipilot/modules/parking/controller/avp_control_debug --record $dir 
    for item in "$dir"/*; do
        if [[ "$item" == *.json ]]; then
            python $pwd/report_generator.py --file-path $item
        fi
    done
done | sort -u

if ls *.txt 1> /dev/null 2>&1; then
    rm *.txt
fi