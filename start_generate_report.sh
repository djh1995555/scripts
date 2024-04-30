#!/bin/bash
pwd=$(pwd -P)
lwd=$(dirname $pwd)

default_record_dir=$pwd/record/issue_record/84127
default_record_dir=$pwd/record/test_record/2024-04-11/0227/14-56-54

default_record_dir=$pwd/record/test_record/filepath/2024-04-11/0227/15-32-40
# default_record_dir=$pwd/record/test_record/event/2024-04-11/MT227/15-44-26
# default_record_dir=$pwd/record/test_record/download
# default_record_dir=$pwd/record/test_record/adrn_path/2024-04-11/0227/15-32-40
default_record_dir=$pwd/record/test_record/2024-04-23/MT091
record_dir=${1:-$default_record_dir}


plot_signal_filepath=$pwd/report_generator/config/target_signal_lon.yaml
# plot_signal_filepath=$pwd/config/target_signal_lat.yaml



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
    if ! find $dir -maxdepth 1 -type f -name "*.json" -print -quit | grep -q .; then
        echo 'do not have json'
        $lwd/mi_tools/bazel-bin/mipilot/modules/parking/controller/avp_control_debug --record $dir 
    fi
    for item in "$dir"/*; do
        if [[ "$item" == *.json ]]; then
            echo generate report for $item
            python $pwd/report_generator/report_generator.py --file-path $item --target-signal-filepath $plot_signal_filepath
        fi
    done
done | sort -u

if ls dr* 1> /dev/null 2>&1; then
    rm dr*
fi

if ls ins* 1> /dev/null 2>&1; then
    rm ins*
fi