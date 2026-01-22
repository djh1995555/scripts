#!/bin/bash
pwd=$(pwd -P)
lwd=$(dirname $pwd)
dir=$pwd/record/test_record/0124_record
for file in "$dir"/*; do
  if [ -f "$file" ]; then
    filename="${file%.*}"
    mkdir -p "$filename"
    mv "$file" "$filename/"
  fi
done
