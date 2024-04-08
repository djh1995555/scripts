#!/usr/bin/env python
from utils.report_plotter import ReportPlotter
from collections import OrderedDict
from lon_params_estimator.noise_covariance_estimator import *

SAMPLE_TIME = 0.05
SIMULATION_DURATION = 1000 #s

def plot_signal(report_plotter, output_dir, data_dict):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    subplot_figure = None
    plot_html_str = ""
    figure_list = []
    for segment_name, signals in data_dict.items():
        print('segment_name:{}'.format(segment_name))
        legend_list = []
        y_list = []
        x_list = []
        for signal_name, data in signals.items():
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
    with open(os.path.join(output_dir,'output.html'), 'w') as f:
        f.write(html_str)

class NoiseGenerator(object):
    def __init__(self,length):
        self._noise_num = 4

        input_1_noise_deviation = math.sqrt(0.1)
        input_2_noise_deviation = math.sqrt(0.2)
        input_3_noise_deviation = math.sqrt(0.4)
        output_noise_deviation = math.sqrt(1.0)
        self._noise_1 = np.random.normal(0, input_1_noise_deviation, size=(length,))
        self._noise_2 = np.random.normal(0, input_2_noise_deviation, size=(length,))
        self._noise_3 = np.random.normal(0, input_3_noise_deviation, size=(length,))
        self._output_noise = np.random.normal(0, output_noise_deviation, size=(length,))

        t = [SAMPLE_TIME * i for i in range(length)]
        self._input_1_true = np.sin(np.dot(2*math.pi*0.006, t)) * np.sin(np.dot(2*math.pi*0.006/3.3, t)) 
        self._input_2_true = np.sin(np.dot(2*math.pi*0.012, t)) * np.sin(np.dot(2*math.pi*0.012/3.3, t)) 
        self._input_3_true = np.sin(np.dot(2*math.pi*0.014, t)) * np.sin(np.dot(2*math.pi*0.014/3.3, t)) 
        self._output_true = []

        self._input_1 = []
        self._input_2 = []
        self._input_3 = []
        self._output = []
        for i in range(length):
            self._input_1.append(self._input_1_true[i] + self._noise_1[i])
            self._input_2.append(self._input_2_true[i] + self._noise_2[i])
            self._input_3.append(self._input_3_true[i] + self._noise_3[i])

            theta = np.array([1,2,3])
            # if(i > int(length/2)):
            #     theta = np.array([2,2,3])
            output = np.dot(theta.T,np.array([self._input_1_true[i],self._input_2_true[i],self._input_3_true[i]]))
            self._output_true.append(output)
            self._output.append(output + self._output_noise[i])

    def get_input(self, i):
        return [self._input_1[i],
                self._input_2[i],
                self._input_3[i],
                self._output[i]]

    def get_total_input(self):
        return [self._input_1, self._input_2, self._input_3, self._output_noise]

    def get_total_input_true(self):
        return [self._input_1_true, self._input_2_true, self._input_3_true, self._output_true]

    def get_noise_num(self):
        return self._noise_num
    
def main():
    report_plotter = ReportPlotter('ReportGenerator')
    signal_length = int(SIMULATION_DURATION / SAMPLE_TIME)
    noise_generator = NoiseGenerator(signal_length)
    poly_order = 4
    wl = 30
    wr = 30
    forget_factor = 0.995
    noise_covariance_estimator = NoiseCovarianceEstimator(noise_generator.get_noise_num(), poly_order, wl, wr, forget_factor)
    noise_covariance_0 = []
    noise_covariance_1 = []
    noise_covariance_2 = []
    noise_covariance_3 = []
    for i in range(signal_length):
        noise_covariance = noise_covariance_estimator.update(noise_generator.get_input(i))
        noise_covariance_0.append(noise_covariance[0,0])
        noise_covariance_1.append(noise_covariance[1,1])
        noise_covariance_2.append(noise_covariance[2,2])
        noise_covariance_3.append(noise_covariance[3,3])

    output_data = OrderedDict()

    covariance_plot = OrderedDict()
    covariance_plot['noise_covariance_0'] = noise_covariance_0
    covariance_plot['noise_covariance_1'] = noise_covariance_1
    covariance_plot['noise_covariance_2'] = noise_covariance_2
    covariance_plot['noise_covariance_3'] = noise_covariance_3
    output_data['covariance_plot'] = covariance_plot

    input_signal_plot = OrderedDict()
    input_data = noise_generator.get_total_input_true()
    input_signal_plot['input_0'] = input_data[0]
    input_signal_plot['input_1'] = input_data[1]
    input_signal_plot['input_2'] = input_data[2]
    input_signal_plot['output'] = input_data[3]
    output_data['input_signal_plot'] = input_signal_plot    

    input_signal_with_noise_plot = OrderedDict()
    input_data = noise_generator.get_total_input()
    input_signal_with_noise_plot['input_with_noise_0'] = input_data[0]
    input_signal_with_noise_plot['input_with_noise_1'] = input_data[1]
    input_signal_with_noise_plot['input_with_noise_2'] = input_data[2]
    input_signal_with_noise_plot['output_with_noise'] = input_data[3]
    output_data['input_signal_with_noise_plot'] = input_signal_with_noise_plot 

    estimation_plot = OrderedDict()
    delayed_y = noise_covariance_estimator.get_kalman_delayed_y()
    smoothed_y = noise_covariance_estimator.get_kalman_smoothed_y()
    estimation_plot['input_with_noise_0'] = input_data[0]
    print('input shape:{}, delayed shape:{}'.format(input_data[1][0:10], delayed_y[1][0:10]))
    estimation_plot['delayed_y_0'] = delayed_y[0]
    estimation_plot['smoothed_y_0'] = smoothed_y[0]
    estimation_plot['delayed_y_1'] = delayed_y[1]
    estimation_plot['smoothed_y_1'] = smoothed_y[1]
    estimation_plot['delayed_y_2'] = delayed_y[2]
    estimation_plot['smoothed_y_2'] = smoothed_y[2]
    estimation_plot['delayed_y_3'] = delayed_y[3]
    estimation_plot['smoothed_y_3'] = smoothed_y[3]
    output_data['estimation_plot'] = estimation_plot 
    dir_path = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__))))

    plot_signal(report_plotter, os.path.join(dir_path,'output','noise'), output_data)


if __name__ == '__main__':
    main()