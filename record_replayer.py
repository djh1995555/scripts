#!/usr/bin/env python
import argparse
import os
import subprocess
import sys
import time

def main(args):
    target_channels = ['/parking/controller/command','/parking/controller/debug','/parking/controller/hmi_report','/parking/controller/status']
    process_list = []

    print(f"replay_record_filepath:{args.target_record}")
    print(f"output_dir:{args.output_dir}")

    control_node_cmd = [f"{args.project_dir}/bazel-bin/mipilot/modules/parking/controller/app/node_launch_controller", 
                        f"{args.project_dir}/mipilot/conf/cross_platform/parking/controller/node_launch_controller_debug.pb.conf"]
    control_process = subprocess.Popen(control_node_cmd, cwd = args.project_dir)
    process_list.append(control_process)


    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir) 
    output_name = os.path.join(args.output_dir, "replay.record")

    # record_process = subprocess.Popen(['recorder', 'record', '-o', output_name, '-a',])
    record_cmd = ['recorder', 'record', '-o', output_name]
    for target_channel in target_channels:
        record_cmd.append('-c')
        record_cmd.append(target_channel) 
    record_process = subprocess.Popen(record_cmd)
    process_list.append(record_process)


    play_cmd = ['recorder', 'play', '-f', args.target_record]
    play_cmd.append('-k')
    for target_channel in target_channels:
        play_cmd.append(target_channel)  
    record_play_process = subprocess.Popen(play_cmd)
    process_list.append(record_play_process)

    while(True):     
        time.sleep(1) 
        print("poll:{}".format(record_play_process.poll())) 
        if(record_play_process.poll() is not None):
            record_process.terminate()
            control_process.terminate()
            time.sleep(1) 
            sys.exit()

if __name__ == '__main__':
    parser = argparse.ArgumentParser('Record Replayer')
    scripts_dirname = os.path.dirname(os.path.abspath(__file__))
    parser.add_argument('--target-record', type=str)
    parser.add_argument('--output-dir', default=os.path.join(scripts_dirname,'record/replay_record/default_report'))
    parser.add_argument('--project-dir', default='/home/mi/tmp_repo_mipilot_mbf_debug_9645_tzfrni')
    parser.add_argument('--time', default='0000', type=str)
    args = parser.parse_args()
    
    main(args)