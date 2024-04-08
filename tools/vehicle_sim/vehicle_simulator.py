#!/usr/bin/env python
import argparse
import datetime
import math
import os
import shutil
import numpy as np
import pandas as pd
from collections import OrderedDict 
import yaml
from utils.report_plotter import ReportPlotter
from dynamic_model.vehicle_dynamic_model import *
from controller.vehicle_controller import *
from utils.signal_generator import *

def plot_signal(report_plotter, output_dir, data_dict, name = 'output'):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    src_config = os.path.join(dir_path,'config',args.config)
    tgt_config = os.path.join(output_dir,args.config)
    shutil.copyfile(src_config, tgt_config) 

    subplot_figure = None
    plot_html_str = ""
    figure_list = []
    for segment_name, signals in data_dict.items():
        # print('segment_name:{}'.format(segment_name))
        legend_list = []
        y_list = []
        x_list = []
        for signal_name, data in signals.items():
            # print(' segment_name:{}, len:{}'.format(segment_name, len(data)))
            legend_list.append(signal_name)
            y_list.append(np.array(data))
            x_list = np.array(range(len(data)))
        subplot = report_plotter.plot_figure_plotly(x_list = x_list, 
                                                    y_list = y_list,
                                                    legend_list = legend_list,
                                                    x_label = 'timestamp',
                                                    y_label = 'value',
                                                    title = '{}'.format(segment_name),
                                                    legend_prefix = '',
                                                    figure_height= 300,)
        figure_list.append(subplot)

    subplot_figure_list = [(i + 1, 1, fig) for i, fig in enumerate(figure_list)]
    subplot_figure = report_plotter.append_figure_to_subplot_plotly(subplot_figure_list, 
                                                                        row_num = len(figure_list), 
                                                                        col_num = 1, 
                                                                        template="plotly_dark", 
                                                                        subplot_fig=subplot_figure,
                                                                        figure_height = 300,
                                                                        vertical_spacing = 0.02)
    plot_html_str += report_plotter.get_fuel_fig_html_str({'output_name': subplot_figure})
    html_str = report_plotter.generate_html_fuel_report(plot_html_str)
    with open(os.path.join(output_dir,'{}.html'.format(name)), 'w') as f:
        f.write(html_str)

def main(args):
    config_filepath = os.path.join(dir_path,'config', args.config)
    with open(config_filepath, 'r') as f:
        config = yaml.load(f)
    report_plotter = ReportPlotter('ReportGenerator')
    simulation_time = config['simulation_time']
    sample_time = config['vehicle_param']['sample_time']
    init_v = config['vehicle_param']['init_v']
    ref_length = config['ref_length']

    signal_generator = SignalGenerator(simulation_time, sample_time, init_v + 5, ref_length, config['vehicle_param'])
    vehicle_dynamic_model = VehicleDynamicModel(config['vehicle_param'], signal_generator.get_pitch(0))
    vehicle_controller = VehicleController(config)

    output_data = OrderedDict()
    ref_v_data = []
    actual_v_data = []
    estimated_v_data = []
    pitch_data = []
    pitch_measured_data = []
    traction_data = []
    traction_measured_data = []
    target_acc_data = []
    acc_measured_data = []
    vehicle_mass_estimated_data = []
    drag_coefficient_estimated_data = []
    rolling_coefficient_estimated_data = []
    vehicle_mass_gt_data = []
    drag_coefficient_gt_data = []
    rolling_coefficient_gt_data = []
    noise_covariance_0 = []
    noise_covariance_1 = []
    noise_covariance_2 = []
    noise_covariance_3 = []
    state_covariance_0 = []
    state_covariance_1 = []
    state_covariance_2 = []
    K_0 = []
    K_1 = []
    K_2 = []
    phi_0 = []
    phi_1 = []
    phi_2 = []
    estimated_err_data = []
    for i in range(int(simulation_time/sample_time)):
        ref_v = signal_generator.get_ref_v(i)
        pitch = signal_generator.get_pitch(i)
        ca = signal_generator.get_ca(i)
        cr = signal_generator.get_cr(i)
        controller_input = vehicle_dynamic_model.get_state()
        traction = vehicle_controller.compute_control_cmd(ref_v, controller_input)
        vehicle_dynamic_model.update(traction, pitch, drag_coefficient=ca, rolling_friction_coefficient=cr)
        ref_v_data.append(ref_v[0])
        actual_v_data.append(controller_input[V])
        pitch_data.append(pitch)
        pitch_measured_data.append(controller_input[PITCH])
        traction_data.append(traction)
        traction_measured_data.append(controller_input[TRACTION])
        acc_measured_data.append(controller_input[A_REPORT])
        target_acc_data.append(vehicle_controller.get_target_acc())
        estimated_v_data.append(vehicle_controller.get_v_estimated())
        vehicle_mass_estimated_data.append(vehicle_controller.get_mass_estimated())
        drag_coefficient_estimated_data.append(vehicle_controller.get_drag_coefficient_estimated())
        rolling_coefficient_estimated_data.append(vehicle_controller.get_rolling_coefficient_estimated())
        vehicle_mass_gt_data.append(config['vehicle_param']['vehicle_mass']/vehicle_controller.get_weight_scale())
        drag_coefficient_gt_data.append(ca)
        rolling_coefficient_gt_data.append(cr)

        K = vehicle_controller.get_K()
        P = vehicle_controller.get_state_covariance()
        noise_covariance = vehicle_controller.get_noise_covariance()
        phi = vehicle_controller.get_phi()

        K_0.append(K[0,0])    
        state_covariance_0.append(P[0,0])    
        noise_covariance_0.append(noise_covariance[0,0])
        noise_covariance_1.append(noise_covariance[1,1])
        phi_0.append(phi[0,0])
        if(config['controller_params']['lon_params_estimator_type'] != 'RLS4'):
            K_1.append(K[1,0])
            state_covariance_1.append(P[1,1])
            noise_covariance_2.append(noise_covariance[2,2])
            phi_1.append(phi[0,1])
            if(config['controller_params']['lon_params_estimator_type'] == 'RLS'):
                K_2.append(K[2,0])
                state_covariance_2.append(P[2,2])
                noise_covariance_3.append(noise_covariance[3,3])
                phi_2.append(phi[0,2])
            
        estimated_err_data.append(vehicle_controller.get_err())
        # print('Plot Progress: {}/{}'.format(i, int(simulation_time/sample_time)))

    mass_plot = OrderedDict()
    mass_plot['mass_estimated'] = vehicle_mass_estimated_data
    mass_plot['mass_gt'] = vehicle_mass_gt_data
    output_data['mass_plot'] = mass_plot
    
    drag_coefficient_plot = OrderedDict()
    drag_coefficient_plot['ca_estimated'] = drag_coefficient_estimated_data
    drag_coefficient_plot['ca_gt'] = drag_coefficient_gt_data
    output_data['drag_coefficient_plot'] = drag_coefficient_plot

    rolling_coefficient_plot = OrderedDict()
    rolling_coefficient_plot['cr_estimated'] = rolling_coefficient_estimated_data
    rolling_coefficient_plot['cr_gt'] = rolling_coefficient_gt_data
    output_data['rolling_coefficient_plot'] = rolling_coefficient_plot

    phi_plot = OrderedDict()
    phi_plot['phi_0'] = phi_0

    state_covariance_plot = OrderedDict()
    state_covariance_plot['state_covariance_0'] = state_covariance_0
        
    noise_covariance_plot = OrderedDict()
    noise_covariance_plot['noise_covariance_0'] = noise_covariance_0
    noise_covariance_plot['noise_covariance_1'] = noise_covariance_1
        
    state_gain_plot = OrderedDict()
    state_gain_plot['K_0'] = K_0
    
    if(config['controller_params']['lon_params_estimator_type'] != 'RLS4'):
        state_gain_plot['K_1'] = K_1
        noise_covariance_plot['noise_covariance_2'] = noise_covariance_2
        state_covariance_plot['state_covariance_1'] = state_covariance_1
        phi_plot['phi_1'] = phi_1
        if(config['controller_params']['lon_params_estimator_type'] == 'RLS'):
            state_gain_plot['K_2'] = K_2
            noise_covariance_plot['noise_covariance_3'] = noise_covariance_3
            state_covariance_plot['state_covariance_2'] = state_covariance_2
            phi_plot['phi_2'] = phi_2
            
    output_data['state_covariance_plot'] = state_covariance_plot        
    output_data['noise_covariance_plot'] = noise_covariance_plot
    
    output_data['state_gain_plot'] = state_gain_plot
    output_data['phi_plot'] = phi_plot

    estimated_err_plot = OrderedDict()
    estimated_err_plot['estimated_err'] = estimated_err_data
    output_data['estimated_err_plot'] = estimated_err_plot

    velocity_plot = OrderedDict()
    velocity_plot['reference_v'] = ref_v_data
    velocity_plot['actual_v'] = actual_v_data
    velocity_plot['estimated_v'] = estimated_v_data
    output_data['velocity_plot'] = velocity_plot

    acc_plot = OrderedDict()
    acc_plot['target_acc'] = target_acc_data
    acc_plot['acc_measured'] = acc_measured_data
    output_data['acc_plot'] = acc_plot

    traction_plot = OrderedDict()
    traction_plot['traction_gt'] = traction_data
    traction_plot['traction_measured'] = traction_measured_data
    output_data['traction_plot'] = traction_plot

    pitch_plot = OrderedDict()
    pitch_plot['pitch_gt'] = pitch_data
    pitch_plot['pitch_measured'] = pitch_measured_data
    output_data['pitch_plot'] = pitch_plot
    
    if(config['vehicle_param']['const_ref_v']):
        ref_v_type = 'const_ref_v'
    else:
        ref_v_type = 'variable_ref_v'
    plot_signal(report_plotter, args.output_dir,output_data,
                '{}-{}-{}'.format(ref_v_type, config['controller_params']['lon_params_estimator_type'],config['controller_params']['lon_params_estimator_params']['rls_type']))


if __name__ == '__main__':
    parser = argparse.ArgumentParser('Sim Vehicle Dynamic Model')
    dir_path = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__))))
    # print('file path:{}'.format(dir_path))
    parser.add_argument('--config', default='simulator_config.yaml', type=str)
    parser.add_argument('--output-dir', default=os.path.join(dir_path,'output','{}'.format(datetime.datetime.now())))
    args = parser.parse_args()
  
    main(args)