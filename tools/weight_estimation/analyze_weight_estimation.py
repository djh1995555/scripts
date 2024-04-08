#!/usr/bin/env python
import argparse
import datetime
import math
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

WEIGHT_GT = 'weight_gt'
def main(args):
    data = pd.read_csv(os.path.join(args.target_dir,args.target_filename))
    print(data)
    output_dir = os.path.join(args.target_dir, str(datetime.datetime.now()))
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    sorted_data = data.sort_values(by=WEIGHT_GT, ascending=True)
    weight_gt = sorted_data[WEIGHT_GT]
    df = sorted_data.drop([WEIGHT_GT,'bag_name'], axis=1)
    
    df_error = pd.DataFrame()
    for idx, column_name in enumerate(df.columns):
        column_data = df[column_name]
        print(column_data)
        print(weight_gt)
        error_data = (column_data - weight_gt) / weight_gt * 100
        data_name = 'error_of_{}'.format(column_name)
        df_error[data_name] = error_data
        
        # plt.figure(idx)
        # plt.plot(weight_gt, error_data,'r.')
        # plt.grid()
        # max_value = max(abs(error_data)) + 10
        # plt.ylim([-max_value,max_value])
        # plt.xlabel(WEIGHT_GT)
        # plt.ylabel('error (%)')
        # plt.title(data_name)
        # plt.savefig(os.path.join(output_dir,'{}.png'.format(data_name)))
        
        # max_value = max(abs(error_data)) + 10
        # histogram_interval = 5
        # max_value = (int(max_value / histogram_interval) + 1) * histogram_interval
        # bins = np.arange(-max_value, max_value, histogram_interval)
        # hist, bins = np.histogram(error_data, bins=bins)
        # fig = plt.figure(figsize=(8, 6))
        # ax = fig.subplots()
        # bars = ax.bar(bins[:-1], hist, width=(bins[1]-bins[0]), align='edge', edgecolor='black')
        # for bar in bars:
        #     height = bar.get_height()
        #     if(height == 0):
        #         continue
        #     plt.text(bar.get_x() + bar.get_width() / 2, height + 1, round(height, 2), ha='center', color='black', fontsize=10)
        # ax.set_title('Distribution of Random Data')
        # ax.set_xlabel('Value')
        # ax.set_ylabel('Frequency')
        # ax.grid()
        # fig.savefig(os.path.join(output_dir,'{}.png'.format(data_name)))

    df_error_list = []
    df_error_of_end_columns = [col for col in df_error.columns if 'error_of_end' in col]
    df_error_list.append(df_error[df_error_of_end_columns])
    df_error_of_mean_columns = [col for col in df_error.columns if 'error_of_mean' in col]
    df_error_list.append(df_error[df_error_of_mean_columns]) 
    
    plot_num = df_error_list[0].shape[1]
    row_num = 3
    column_num = 3
    num_fig_plots = int(plot_num / (row_num * column_num)) + 1
    print(plot_num, num_fig_plots)
    num_small_plots_per_fig = row_num * column_num
     
    i = 0   
    for df in df_error_list:
        for fig_plot_index in range(num_fig_plots):
            fig, axs = plt.subplots(row_num, column_num, figsize=(21, 12))
            for row in range(row_num):
                for col in range(column_num):
                    small_plot_index = fig_plot_index * num_small_plots_per_fig + row * column_num + col
                    if small_plot_index < plot_num:
                        ax = axs[row, col]
                        column_name = df.columns[small_plot_index]
                        max_value = max(abs(df[column_name])) + 10
                        ax.plot(weight_gt, df[column_name],'b.')
                        ax.set_ylim([-max_value,max_value])
                        ax.set_xlabel(WEIGHT_GT)
                        ax.set_ylabel('error (%)')
                        ax.set_title(column_name)
                        ax.grid()
                    else:
                        axs[row, col].axis('off')
            
            plt.tight_layout()
            plt.savefig(os.path.join(output_dir,'scartter_result_{}.png'.format(i)))
            i += 1

    i = 0
    for df in df_error_list:
        for fig_plot_index in range(num_fig_plots):
            fig, axs = plt.subplots(row_num, column_num, figsize=(21, 12))
            for row in range(row_num):
                for col in range(column_num):
                    small_plot_index = fig_plot_index * num_small_plots_per_fig + row * column_num + col
                    if small_plot_index < plot_num:
                        ax = axs[row, col]
                        column_name = df.columns[small_plot_index]
                        histogram_interval = 2
                        # max_value = max(abs(df[column_name])) + 10
                        # max_value = (int(max_value / histogram_interval) + 1) * histogram_interval
                        max_value = 40
                        bins = np.arange(-max_value, max_value, histogram_interval)
                        hist, bins = np.histogram(df[column_name], bins=bins)
                        ax.bar(bins[:-1], hist, width=(bins[1]-bins[0]), align='edge', edgecolor='black')
                        ax.set_title(column_name)
                        ax.set_xlabel('Error')
                        ax.set_ylabel('Frequency')
                        ax.grid()
                    else:
                        axs[row, col].axis('off')
            plt.tight_layout()
            plt.savefig(os.path.join(output_dir,'histogram_result_{}.png'.format(i)))
            i += 1

if __name__ == '__main__':
    parser = argparse.ArgumentParser('weight comparison')
    file_dirname = os.path.dirname(os.path.abspath(__file__))
    parser.add_argument('--target-dir', default=os.path.join(file_dirname,'analysis_output'))
    parser.add_argument('--target-filename', default='weight_estimation_analysis_result.csv')
    args = parser.parse_args()

    main(args)