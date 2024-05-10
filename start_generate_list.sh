#!/bin/bash
pwd=$(pwd -P)
lwd=$(dirname $pwd)

record_dir=/home/mi/debug/scripts/record/test_record/2024-04-15/MT091
declare -a result_dirs
record_dir_list="record_dirs.lst"

function traverse_directory() {
    local dir=$1
    local has_record_file=false

    for item in "$dir"/*; do
        filename=$(basename $item)
        if [ -d "$item" ]; then
            traverse_directory "$item"
        elif [[ "$filename" == *.record || "$filename" == full_record.* ]]; then
            result_dirs+=("$dir")
            echo $dir >> $record_dir_list
            has_record_file=true
            break
        fi
    done

    if [ "$has_record_file" = false ]; then
        return
    fi
}

traverse_directory $record_dir