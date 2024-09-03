#!/bin/bash

# 定义目标文件夹路径
target_folder="/home/mi/debug/scripts/record/test_record/2024-07-22"

# 递归查找目标文件夹中的文件，并将后缀名为.mcap的文件改成.record
find "$target_folder" -type f -name "*.mcap" | while read file; do
    new_file="${file%.mcap}.record"
    mv "$file" "$new_file"
    echo "文件 $file 已重命名为 $new_file"
done

echo "处理完成"