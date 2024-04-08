#!/usr/bin/env python
from utils.report_plotter import ReportPlotter
from collections import OrderedDict
from lon_params_estimator.noise_covariance_estimator import *
from scipy import signal

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

class SignalGenerator(object):
    def __init__(self,length,segment_length):
        self._segment_length = segment_length
        delay_frame = segment_length
        t = [SAMPLE_TIME * i for i in range(length)]
        self._original_signal = np.sin(np.dot(2*math.pi*0.006, t)) * np.sin(np.dot(2*math.pi*0.006/3.3, t)) 
        tmp_signal = 2 * self._original_signal
        threshold = -0.5
        tmp_signal[tmp_signal < threshold] = threshold
        self._delayed_signal = np.roll(tmp_signal, delay_frame)
        self._delayed_signal[:delay_frame] = 0.0

    def get_original_signal(self):
        return self._original_signal
    
    def get_delayed_signal(self):
        return self._delayed_signal
    
    def get_original_signal_segment(self, i):
        return self._original_signal[i:i+self._segment_length]

    def get_delayed_signal_segment(self, i):
        return self._delayed_signal[i:i+self._segment_length]
        
def time_delay_estimation(x,y):
    n = x.shape[0]
    corr = signal.correlate(y, x, mode='same') / np.sqrt(signal.correlate(x, x, mode='same')[int(n/2)] * signal.correlate(y, y, mode='same')[int(n/2)])
    delay_arr = np.linspace(-0.5*n, 0.5*n, n)
    delay = delay_arr[np.argmax(corr)]
    return delay

def main():
    report_plotter = ReportPlotter('ReportGenerator')
    signal_length = int(SIMULATION_DURATION / SAMPLE_TIME)
    segment_length = 500
    signal_generator = SignalGenerator(signal_length, segment_length)

    original_signal = signal_generator.get_original_signal()
    delayed_signal = signal_generator.get_delayed_signal()

    time_delay = time_delay_estimation(original_signal,delayed_signal)
    print('time_delay:{}'.format(time_delay))

    time_delay_data = []
    # for i in range (signal_length - segment_length):
    #     original_signal_segment = signal_generator.get_original_signal_segment(i)
    #     delayed_signal_segment = signal_generator.get_delayed_signal_segment(i)
    #     time_delay_data.append(time_delay_estimation(original_signal_segment,delayed_signal_segment))

    output_data = OrderedDict()

    signal_plot = OrderedDict()
    signal_plot['original_signal'] = original_signal
    signal_plot['delayed_signal'] = delayed_signal
    output_data['signal_plot'] = signal_plot

    delay_time_plot = OrderedDict()
    delay_time_plot['delay_time'] = time_delay_data
    output_data['delay_time_plot'] = delay_time_plot

    dir_path = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__))))

    plot_signal(report_plotter, os.path.join(dir_path,'output','time_delay'), output_data)


if __name__ == '__main__':
    main()