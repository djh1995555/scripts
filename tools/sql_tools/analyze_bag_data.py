#!/usr/bin/env python
import argparse
import os
import pandas as pd
import shutil

LON_BEHAVIOR_THRESHOLD = 90
LAT_BEHAVIOR_THRESHOLD = 90
LOW_SPEED_THRESHOLD = 80
HIGH_SPEED_THRESHOLD = 85

def filter_data(df):
    # print(df)
    error = df['Avg Speed(km/h)'] - df['Avg Trip Speed(km/h)']
    mask =  (error >= -1) & (error <= 1) & \
            (df['Auto(%)'] >= 99) & \
            (df['Auto(%)'] <= 100) & \
            (df['Leading Ratio(%)'] <= 10000) & \
            (df['Coasting Ratio(%)'] <= 100) & \
            (df['FE Source Ratio(%)'] <= 100) & \
            (df['Avg Speed(km/h)'] <= 100)
    df = df[mask]
    # print(df)
    return df


def select_data_by_item(df, item1, item2, split_point, step_ranges):
    result = pd.DataFrame()
    
    high_part = df[df[item1] > split_point[2]]
    low_part = df[(df[item1] > split_point[0]) & (df[item1] < split_point[1])]
    
    # first_step_range = step_ranges[0]
    # second_step_range = step_ranges[1]
    
    # nlargest_part = high_part.nlargest(first_step_range, item1)
    # df_max = nlargest_part.nlargest(second_step_range, item2)
    # result = pd.concat([result, df_max],ignore_index=True)

    # df_max_a_min_b = nlargest_part.nsmallest(second_step_range, item2) 
    # result = pd.concat([result, df_max_a_min_b],ignore_index=True)
    
    # nsmallest_part = low_part.nsmallest(first_step_range, item1)
    # df_min = nsmallest_part.nsmallest(second_step_range, item2)
    # result = pd.concat([result, df_min],ignore_index=True)

    # df_min_a_max_b = nsmallest_part.nlargest(second_step_range, item2)
    # result = pd.concat([result, df_min_a_max_b],ignore_index=True)
    
    second_step_range = step_ranges[1]
    df_max = high_part.nlargest(second_step_range, item2)
    result = pd.concat([result, df_max],ignore_index=True)

    df_max_a_min_b = high_part.nsmallest(second_step_range, item2) 
    result = pd.concat([result, df_max_a_min_b],ignore_index=True)
    

    df_min = low_part.nsmallest(second_step_range, item2)
    result = pd.concat([result, df_min],ignore_index=True)

    df_min_a_max_b = low_part.nlargest(second_step_range, item2)
    result = pd.concat([result, df_min_a_max_b],ignore_index=True)
    return result 
  
def select_data(df):
    # leading_split = [2500,5000,750]
    # result_by_leading_info = select_data_by_item(df,'Leading Ratio(%)','Pitch Square',leading_split,[15,6])
    # pitch_split = [0,500,500]
    # result_by_pitch = select_data_by_item(df,'Pitch Square','Leading Ratio(%)',pitch_split,[15,6])
    # return pd.merge(result_by_leading_info, result_by_pitch, how='inner')
    
    leading_split = [2000,4000,7500]
    result_by_leading_info = select_data_by_item(df,'Leading Ratio(%)','Pitch Square',leading_split,[9,3])
    return result_by_leading_info

    
def main(args):
    data_source = os.path.join(file_dirname,'data',args.data_source)
    df = pd.read_csv(data_source)
    df = filter_data(df)
    lon_df = df[df['FE Source Ratio(%)'] > LON_BEHAVIOR_THRESHOLD]
    lat_df = df[df['FE Source Ratio(%)'] < LAT_BEHAVIOR_THRESHOLD]
    df_dict = {}
    
    df_dict['lon_low_speed'] = lon_df[df['Avg Speed(km/h)'] < LOW_SPEED_THRESHOLD]
    df_dict['lat_low_speed'] = lat_df[df['Avg Speed(km/h)'] < LOW_SPEED_THRESHOLD]
    
    # df_dict['lon_medium_speed'] = lon_df[(df['Avg Speed(km/h)'] > LOW_SPEED_THRESHOLD) & (df['Avg Speed(km/h)'] < HIGH_SPEED_THRESHOLD)]
    # df_dict['lat_medium_speed'] = lat_df[(df['Avg Speed(km/h)'] > LOW_SPEED_THRESHOLD) & (df['Avg Speed(km/h)'] < HIGH_SPEED_THRESHOLD)]
    
    df_dict['lon_high_speed'] = lon_df[df['Avg Speed(km/h)'] > HIGH_SPEED_THRESHOLD]
    df_dict['lat_high_speed'] = lat_df[df['Avg Speed(km/h)'] > HIGH_SPEED_THRESHOLD]

    output_folder = os.path.join(file_dirname,args.output_folder)
    if os.path.exists(output_folder):
        shutil.rmtree(output_folder)
    os.makedirs(output_folder) 
        
    all_result = pd.DataFrame() 
    for name, df in df_dict.items():
        print('==============name = {}=================='.format(name))
        df.to_csv(os.path.join(output_folder,name+'_original.csv'))
        result = select_data(df)
        result.to_csv(os.path.join(output_folder,name+'.csv'))
        all_result = pd.concat([all_result, result],ignore_index=True)
        all_result.to_csv(os.path.join(output_folder,'all_result.csv'))
        
    
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser('Bag data analyzer')
    file_dirname = os.path.dirname(os.path.abspath(__file__))
    parser.add_argument('--data-source', default='data.csv', type=str)
    parser.add_argument('--output-folder', default='analyzed_data', type=str)
    
    args = parser.parse_args()
    main(args)