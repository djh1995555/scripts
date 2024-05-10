#!/usr/bin/env python
import argparse
import os
import subprocess
import sys
import time

def main(args):
    target_channels_blocked = ['/parking/controller/command','/parking/controller/debug','/parking/controller/hmi_report','/parking/controller/status']
    record_target_channels = [
      "/localization/kinetic_localization",
      "/sensors/chassis/parking",
      "/parking/apa/trajectory",
      "/parking/avp/trajectory",
      "/sensors/ultrasonic/echo",
      "/parking/obstacle_map/apa",
      "/hmi/parking/interact_command",
      "/parking/apa/apa_map",
      "/parking/avp/state_machine",
      "/parking/apa/state_machine",
      "/parking/apa_planner/status",
      "/parking/apa_planner/hmi_report",
      "/perception/prediction/obstacles/avp",
      "/mcu/soc/carconfig_params",
      "/parking/controller/command",
      "/parking/controller/debug",
      "/parking/controller/status",
      "/parking/controller/hmi_report",
    ]
    process_list = []


    file_paths = list(os.walk(args.target_record_dir))
    target_record_list = []
    for root, dirs, files in file_paths:
        for file_name in files:
            if(file_name.startswith("full_record.") or file_name.endswith(".record")):
                target_record_list.append(os.path.join(root, file_name))

    print(f"replay_record_filepath:{args.target_record_dir}")
    print(f"output_dir:{args.output_dir}")
    print(f"target_record_list:{target_record_list}")

    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir) 

    play_cmd = ['recorder', 'play', '-f']
    for target_record in target_record_list:
        play_cmd.append(target_record) 
    play_cmd.append('-k')
    for target_channel in target_channels_blocked:
        play_cmd.append(target_channel)  
    if(args.start_time is not "00"):
        play_cmd.append('-s')
        play_cmd.append(str(args.start_time))
    print(f"play_cmd=========={play_cmd}")
    record_play_process = subprocess.Popen(play_cmd)
    process_list.append(record_play_process)
    time.sleep(1.9) 

    control_node_cmd = [f"{args.project_dir}/bazel-bin/mipilot/modules/parking/controller/app/node_launch_controller", 
                        f"{args.project_dir}/mipilot/conf/cross_platform/parking/controller/node_launch_controller_debug.pb.conf"]
    control_process = subprocess.Popen(control_node_cmd, cwd = args.project_dir)
    process_list.append(control_process)


    output_name = os.path.join(args.output_dir, "replay.record")
    # record_process = subprocess.Popen(['recorder', 'record', '-o', output_name, '-a',])
    record_cmd = ['recorder', 'record', '-o', output_name]
    for target_channel in record_target_channels:
        record_cmd.append('-c')
        record_cmd.append(target_channel) 
    record_process = subprocess.Popen(record_cmd)
    process_list.append(record_process)


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
    parser.add_argument('--target-record-dir', type=str)
    parser.add_argument('--output-dir', default=os.path.join(scripts_dirname,'record/replay_record/default_report'))
    parser.add_argument('--project-dir', default='/home/mi/tmp_repo_mipilot_mbf_1108_NEvh35')
    parser.add_argument('--start-time', default="00", type=str)
    args = parser.parse_args()
    
    main(args)