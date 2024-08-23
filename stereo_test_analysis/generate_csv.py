import os
import pandas as pd

# 读取CSV文件
input_file = 'original_analyze_result.csv'  # 输入文件名
df = pd.read_csv(input_file)

for index, row in df.iterrows():
    add_name = f"{row['日期']}_{row['版本']}_{row['打包号']}_ADD.csv"  # 组合名称
    add2_name = f"{row['日期']}_{row['版本']}_{row['打包号']}_ADD2.csv"  # 组合名称
    new_df = pd.DataFrame(columns=['original_link','城市','日期'])
    new_df.to_csv(os.path.join('original_data',add_name), index=False, encoding='utf-8-sig')
    new_df.to_csv(os.path.join('original_data',add2_name), index=False, encoding='utf-8-sig')
