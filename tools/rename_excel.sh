#!/bin/bash

# 定义目标文件夹路径
target_folder="/home/mi/debug/scripts/stereo_test_excel/original_data"

# 递归查找目标文件夹中的文件，并将后缀名为.mcap的文件改成.record
find "$target_folder" -type f -name "*.csv" | while read file; do
    filename="${file##*/}"
    echo "&&&&&&&&&&&&&&$filename"
    new_file="${target_folder}/2024年$filename"
    mv "$file" "$new_file"
    echo "文件 $file 已重命名为 $new_file"
done

echo "处理完成"