#!/usr/bin/env python
import numpy as np
from plotly.subplots import make_subplots
import plotly.graph_objects as go
from plotly.offline import get_plotlyjs

class ReportPlotter:
    def __init__(self, client):
        self._client = client
        
    def plot_figure_plotly(self, x_list, y_list, legend_list, x_label, y_label,
                        figure_height, legend_prefix, colors=None, title=None, hovertext=None,
                        legend_pos_x=0.0, legend_pos_y=1.2, mode='lines', marker=None,
                        secondary_y=None, y2_label=None, marker_symbol=None,
                        x_range=None, y_range=None):
        if not isinstance(x_list, list):
            x_list = [np.array(x_list), ]
        if not isinstance(y_list, list):
            y_list = [y_list, ]
        if not isinstance(hovertext, list):
            hovertext = [hovertext, ]
        if not isinstance(colors, list):
            colors = [colors, ]
        if len(x_list) == 1:
            x_list = x_list * len(y_list)
        if len(hovertext) == 1:
            hovertext = hovertext * len(y_list)
        if len(colors) == 1:
            colors = colors * len(y_list)
        if not isinstance(legend_list, list):
            legend_list = [legend_list, ]
        if len(y_list) != len(legend_list):
            raise ValueError("dimension mismatching in y_list, color_list, or legend_list")
        if len(x_list) != len(y_list):
            raise ValueError("dimension mismatching in x_list and y_list")
        if x_range is not None:
            x_range = list(x_range)
            if len(x_range) != 2:
                raise ValueError("x_range should be a two-element list")
        if y_range is not None:
            y_range = list(y_range)
            if len(y_range) != 2:
                raise ValueError("y_range should be a two-element list")

        fig = make_subplots(specs=[[{"secondary_y": True}]])
        for idx, (x, y, color, text) in enumerate(zip(x_list, y_list, colors, hovertext)):
            if x.size == 0 or y.size == 0 or \
                    np.isnan(x).all() or np.isnan(y).all():
                continue
            is_secondary_y = False if secondary_y is None else secondary_y[idx]
            fig.add_trace(
                go.Scatter(x=x,
                        y=y,
                        name=legend_prefix + legend_list[idx],
                        line_color=color,
                        hovertext=text,
                        mode=mode,
                        marker=marker,
                        marker_symbol=marker_symbol),
                secondary_y=is_secondary_y)
        x_label = x_label + " <br> initial_x: 0"
        fig.update_layout(height=figure_height,showlegend=True)
        if title is not None:
            fig.update_layout(title=title)
        fig.update_layout(legend={"orientation": "h",
                                "x": legend_pos_x,
                                "y": legend_pos_y})
        # try fixed range if y_range is given
        if x_range:
            fig.update_xaxes(title_text=x_label, range=x_range)
        else:
            fig.update_xaxes(title_text=x_label, autorange=True)
        if y_range and len(y_range) > 0:
            fig.update_yaxes(title_text=y_label, range=y_range, fixedrange=True, secondary_y=False)
        else:
            fig.update_yaxes(title_text=y_label, autorange=True, secondary_y=False)

        if y2_label is not None:
            if y_range:
                fig.update_yaxes(title_text=y2_label, range=y_range, fixedrange=True, secondary_y=False)
            else:
                fig.update_yaxes(title_text=y2_label, autorange=True, secondary_y=False)
        return fig

    def get_fuel_fig_html_str(self, fig_dict):
        if not fig_dict:
            return ""

        fig_plots_str = ""
        for fig_name, fig in sorted(fig_dict.items()):
            fig_plots_str += """
                            <table border="1">
                                    <tr>
                                        <th> {fig_name} </th>
                                    </tr>
                            """.format(fig_name=fig_name)
            fig_plots_str += """
                            <tr>
                            <td style="align:auto" width="{fig_width}"> {fig_str} </td>
                            </tr>
                            """.format(fig_width=fig.layout.width,
                                        fig_str=fig.to_html(include_plotlyjs=False, full_html=False))
            fig_plots_str += "</table>"

        return fig_plots_str

    def generate_html_fuel_report(self, data_figs_str):
        html = """
                <!DOCTYPE html>
                <html>
                    <head>
                        <meta
                            charset="utf-8"
                            http-equiv="Content-Security-Policy"
                            content="default-src 'self';
                                    script-src 'self' 'unsafe-inline' 'unsafe-eval';
                                    frame-src 'self' 'unsafe-inline';
                                    img-src 'self' data: blob:;
                                    style-src 'self' 'unsafe-inline'">
                        <title> Custom Debug Report </title>
                        <script type="text/javascript">{plotlyjs}</script>
                    </head>
                    <body style="color:white;background-color:black">
                    <center>
                        <h1> Custom Debug Report </h1>
                            {data_figs}
                    </center>
                    </body>
                </html>
                        
                """.format(plotlyjs=get_plotlyjs(),
                        data_figs=data_figs_str)
        return html

    def append_figure_to_subplot_plotly(self,
                                        figures,
                                        row_num,
                                        col_num,
                                        figure_width=2000,
                                        figure_height=200,
                                        template="plotly_dark",
                                        subplot_fig=None,
                                        vertical_spacing=0.02):
        if col_num != 1:
            raise NotImplementedError("subplot with more than one columns not supported yet")

        if subplot_fig is None:
            subplot_fig = make_subplots(rows=row_num,
                                        cols=col_num,
                                        vertical_spacing=vertical_spacing,
                                        subplot_titles=[fig["layout"]["title"]["text"] for row, col, fig in figures],
                                        shared_xaxes=True)

        subplot_fig.update_layout(width=figure_width, height=figure_height * row_num, showlegend=True, template=template)

        for row, col, fig in figures:
            # copy data traces
            subplot_fig.add_traces(fig["data"], rows=[row] * len(fig["data"]), cols=[col] * len(fig["data"]))

            # copy xaxis layout for each subplot
            # clear `anchor` and `domain` to make sure these two fields follow the new subplot layout
            fig["layout"]["xaxis"]["anchor"] = None
            fig["layout"]["xaxis"]["domain"] = None
            fig["layout"]["xaxis"]["showticklabels"] = True
            # only show complete xaxis layout for the last subplot
            if row != row_num:
                fig["layout"]["xaxis"]["title"]["text"] = None
            subplot_fig.update_xaxes(fig["layout"]["xaxis"], row=row, col=col)

            # copy yaxis layout for each subplot
            # clear `anchor` and `domain` to make sure these two fields follow the new subplot layout
            fig["layout"]["yaxis"]["anchor"] = None
            fig["layout"]["yaxis"]["domain"] = None
            subplot_fig.update_yaxes(fig["layout"]["yaxis"], row=row, col=col)

        return subplot_fig