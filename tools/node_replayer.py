#!/usr/bin/env python
import os
import sys
import rospy
import rosbag
import fastbag
import signal
import time
import threading
import traceback
import argparse
import subprocess
from utils.bag_finder import BagFinder

def cleanup(children=None, handle_sigint=False):
    alive_child = {p.pid for p in children}

    def signal_handler(sig, frame):
        print("please wait until we kill all launched nodes, remain nodes: {}".format(list(alive_child)))
    if handle_sigint:
        signal.signal(signal.SIGINT, signal_handler)

    def is_alive(pid):
        try:
            os.kill(pid, 0)
            return True
        except ProcessLookupError:
            return False

    def send_signal(pid, sig):
        try:
            print('> Sending {} to {}'.format(sig, pid))
            os.kill(pid, sig)
        except ProcessLookupError:
            pass

    def kill_child(pid):

        print('Shutting down {}'.format(pid))

        if not is_alive(pid):
            return

        send_signal(pid, signal.SIGINT)

        # give it some time to shut down gracefully
        print('[Killing] > Waiting until process {} shuts down'.format(pid))
        start_time = time.time()
        sent_sigterm = False
        sigint_timeout = start_time + 5
        sigterm_timeout = sigint_timeout + 1
        while is_alive(pid):
            time.sleep(0.1)
            if time.time() < sigint_timeout:
                continue

            if not sent_sigterm:
                send_signal(pid, signal.SIGTERM)
                sent_sigterm = True

            if time.time() < sigterm_timeout:
                continue

            send_signal(pid, signal.SIGKILL)
            break

    if len(alive_child) > 1:
        killers = [threading.Thread(target=kill_child, args=(child_name,))
                   for child_name in alive_child]
        [killer.start() for killer in killers]
        [killer.join() for killer in killers]

    print('Cleanup done')
    
def main(args):
    vehicle_name = args.vehicle_name
    target_node = args.target_node
    dir_name_list = [item for argument in args.bagdir for item in argument.split(',') if item != '']
    baglist = BagFinder().find_bags_in_(dir_name_list) 
    print('bag list:{}'.format(baglist))
    
    def signal_handler(*args):
        print('You pressed Ctrl+C!')
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    
    rosnode_process = []
    topic_process = subprocess.Popen(['rostopic', 'list'],
                                stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT)
    result, err = topic_process.communicate()
    if result == 'ERROR: Unable to communicate with master!\n':
        ros_process = subprocess.Popen('roscore')
        rosnode_process.append(ros_process)
        time.sleep(3)
    

    output_dir = os.path.join(args.output_dir)
    if not os.path.exists(output_dir):
        os.makedirs(output_dir) 
    process_fastbag = False
    for count, bag_file in enumerate(baglist):
        subprocess.Popen(['rosparam', 'set', 'use_sim_time', 'true'])
        print('count:{}'.format(count))
        try:
            if ".bag" in bag_file:
                bag_file_name = os.path.basename(bag_file).split('.bag')[0]
                bag = rosbag.Bag(bag_file)
            elif ".db" in bag_file:
                process_fastbag = True
                bag_file_name = os.path.basename(bag_file).split('.db')[0]
                bag = fastbag.Reader(bag_file)
                bag.open()
            else:
                print("incorrect bag {} to read, because it should be .db or .bag ended".format(bag_file))
                continue
        except:
            print("Unable to read bag {}".format(bag_file))
            continue 

        # run control ros node
        if(args.target_node == 'control'):
            control_node_launch_file_path = 'run/control_v2/launch_controller_' + vehicle_name + '.xml'
            control_node_cmd = ['./run/perception/ros_launch.sh', control_node_launch_file_path]
            control_process = subprocess.Popen(control_node_cmd, cwd = args.drive_root)
            process_list.append(control_process)


        # run planning ros node
        if(args.target_node == 'planning'):
            planning_node_launch_file_path = 'run/planning/launch_planning_' + vehicle_name + '.xml'
            planning_node_cmd = ['./run/perception/ros_launch.sh', planning_node_launch_file_path]
            planning_process = subprocess.Popen(planning_node_cmd, cwd = args.drive_root)
            process_list.append(planning_process)

        # run localization ros node
        # localization_node_launch_file_path = 'run/localization/launch_localization_pdb.xml'
        # localization_node_cmd = ['./run/perception/ros_launch.sh', '-d', localization_node_launch_file_path]
        # localization_process = subprocess.Popen(localization_node_cmd, cwd = args.drive_root)
        # process_list.append(localization_process)
                   
        # runtime_plot_process = subprocess.Popen( ['python', './tools/runtime_plotter.py'])
        # process_list.append(runtime_plot_process)
        # time.sleep(3) 
                            
        output_bag = os.path.join(output_dir, 'replay_' + bag_file_name + '.bag')
        
        duration = bag.get_end_time() - bag.get_start_time()
        if not rospy.is_shutdown():
            record_process = subprocess.Popen(['rosbag', 'record', '-O', output_bag, '-a', "--lz4","__name:=my_bag"])
            process_list.append(record_process)
            time.sleep(1.0)
            
            control_cmd_remap = '/vehicle/control_cmd:=/original/vehicle/control_cmd'
            vehicle_state_remap = '/vehicle/status:=/original/vehicle/status'
            planning_remap = '/planning/trajectory:=/original/planning/trajectory'
            lead_info_remap = '/planning/lead_info:=/original/planning/lead_info'
            localization_lidar_localization_topic_name_remap = '/localization/lidar:=/original/localization/lidar'
            localization_gnss_localization_topic_name_remap = '/localization/gnss:=/original/localization/gnss'
            localization_odometry_localization_topic_name_remap = '/localization/odometry:=/original/localization/odometry'
            localization_localization_state_topic_name_remap = '/localization/state:=/original/localization/state'
            localization_localization_state_nav_msg_topic_name_remap = '/localization/state_nav_msg:=/original/localization/state_nav_msg'
            localization_fusion_debug_info_topic_name_remap = '/localization/fusion_debug_info:=/original/localization/fusion_debug_info'
            localization_visual_geometry_localization_topic_name_remap = '/localization/visual_geometry:=/original/localization/visual_geometry'
            if process_fastbag:
                if(args.target_node == 'control'):
                    play_cmd = [
                        'fastbag', 'play', '-i', bag_file, '--clock', '--hz', str(1000), 
                        '--remap', control_cmd_remap, 
                        '--remap', vehicle_state_remap,
                        # '--remap', localization_lidar_localization_topic_name_remap, 
                        # '--remap', localization_gnss_localization_topic_name_remap,
                        # '--remap', localization_odometry_localization_topic_name_remap, 
                        # '--remap', localization_localization_state_topic_name_remap,
                        # '--remap', localization_localization_state_nav_msg_topic_name_remap, 
                        # '--remap', localization_fusion_debug_info_topic_name_remap,
                        # '--remap', localization_visual_geometry_localization_topic_name_remap,
                        ]
                if(args.target_node == 'planning'):
                    play_cmd = [
                        'fastbag', 'play', '-i', bag_file, '--clock', '--hz', str(1000), '--remap', planning_remap, '--remap',lead_info_remap]
            else:
                if(args.target_node == 'control'):
                    play_cmd = [
                        '/opt/plusai/bin/player', '--bags', bag_file, '--clock', '--hz', str(1000), control_cmd_remap, vehicle_state_remap]
                if(args.target_node == 'planning'):
                    play_cmd = [
                        '/opt/plusai/bin/player', '--bags', bag_file, '--clock', '--hz', str(1000), planning_remap, lead_info_remap]
            bagplay_process = subprocess.Popen(play_cmd)
            process_list.append(bagplay_process)
    
            while(True):
                time.sleep(2)
                print("ros shutdown:{}, poll0:{}".format(rospy.is_shutdown(), bagplay_process.poll()))
                if(bagplay_process.poll() is not None):
                    break
            
            cleanup(children=process_list)
            
            print('===================finish replay {}==================='.format(bag_file_name))
            
    while(True):     
        time.sleep(2) 
        print("ros shutdown:{}, poll:{}".format(rospy.is_shutdown(), bagplay_process.poll())) 
        if(bagplay_process.poll() is not None):
            subprocess.Popen(['rosnode', 'kill', '-a'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            time.sleep(2)
            sys.exit()
            
if __name__ == '__main__':
    parser = argparse.ArgumentParser('Node Replayer')
    parser.add_argument('bagdir', nargs='+', type=str, help="path to bag or db (can be more than one, separated by commas/spaces)")
    parser.add_argument('--drive-root', default=os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../drive/opt/relwithdebinfo')))
    parser.add_argument('--output-dir', default=os.path.dirname(os.path.abspath(__file__))+'/replay_node_output', help='output directory of the trucksim test')
    parser.add_argument('--time', default='0', type=str)
    parser.add_argument('--vehicle-name', default='j7-l4e', help='vehicle name used for trucksim simulation')
    parser.add_argument('--target-node', choices=['control','planning'], default='control', type=str)
    args = parser.parse_args()
    rospy.init_node(args.target_node + 'node_replayer', anonymous=True)
    process_list = []
    main(args)