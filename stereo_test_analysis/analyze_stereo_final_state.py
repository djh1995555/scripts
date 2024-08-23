import argparse
from datetime import datetime
import shutil
import pandas as pd
import os
import plotly.graph_objects as go
import matplotlib.pyplot as plt
import warnings
from plotly.subplots import make_subplots
 
# 忽略所有的UserWarning
warnings.filterwarnings('ignore', category=FutureWarning)
warnings.filterwarnings('ignore', category=DeprecationWarning)
DATE = '日期'
VERSION = '版本号'
BAG = '打包'
V_TYPE = '车辆类型'
DATA_LINK = '原始数据链接'
CITY = '城市'
SLOT = '停车场'
PARKING_COUNT = '泊车总次数'
RELEASE_COUNT = '泊车释放次数'
RELEASE_RATE = '泊车释放率(%)'
POSTURE_RIGHT_COUNT = '位姿达标次数'
POSTURE_RIGHT_RATE = '位姿达标率(%)'
HIT_WHEELSTOP_COUNT = '撞轮挡次数'
NOT_HIT_WHEELSTOP_RATE = '不撞轮挡率(%)'
DATA_GROUP = [
	PARKING_COUNT,
	RELEASE_COUNT,
	RELEASE_RATE,
	POSTURE_RIGHT_COUNT,
	POSTURE_RIGHT_RATE,
	HIT_WHEELSTOP_COUNT,
	NOT_HIT_WHEELSTOP_RATE,
	]

def number_to_letter(number):
	return chr(number + 65)

def analyze_data(df, filename):
	# print(filename)
	date, version, bag_id, vehicle_type, city = filename.split('.csv')[0].split('_')
	original_link = None
	if(len(df['original_link'])>0):
		original_link = df['original_link'][0]
	city = None
	if(len(df[CITY])>0):
		city = df[CITY][0]

	analysis_result = {
		DATE: date,
		VERSION: version,
		BAG: bag_id,
		V_TYPE: vehicle_type,
		DATA_LINK: original_link,
		CITY: city,
		}	

	total_count = df.shape[0]
	analysis_result[PARKING_COUNT] = df.shape[0]
	release_count = max(sum(df['库位是否释放'] == 1),sum(df['是否释放'] == 1))
	analysis_result[RELEASE_COUNT] = release_count
	analysis_result[RELEASE_RATE] = f'{(release_count / total_count * 100):.2f}'
	total_reached_count = max(sum(df['位姿达标'] == 1),sum(df['位姿是否达标'] == 1))
	analysis_result[POSTURE_RIGHT_COUNT] = total_reached_count
	analysis_result[POSTURE_RIGHT_RATE] = f'{(total_reached_count / release_count * 100):.2f}'
	total_hit_wheel_stop_count = 0
	if(not df['问题归类'].isna().all()):
		total_hit_wheel_stop_count = sum(((df['是否释放'] == 1) | (df['库位是否释放'] == 1)) & (df['问题归类'].str.count('撞轮挡')))	
	analysis_result[HIT_WHEELSTOP_COUNT] = total_hit_wheel_stop_count
	analysis_result[NOT_HIT_WHEELSTOP_RATE] = f'{(100 - total_hit_wheel_stop_count / release_count * 100):.2f}'

	slots = df['位置'].value_counts()
	idx = 0
	for slot, count in slots.items():
			flag = number_to_letter(idx)
			analysis_result[f'{SLOT}{flag}'] = slot
			analysis_result[f'{flag}{PARKING_COUNT}'] = count
			release_count = sum((df['位置'] == slot) & ((df['是否释放'] == 1) | (df['库位是否释放'] == 1)))
			analysis_result[f'{flag}{RELEASE_COUNT}'] = release_count
			analysis_result[f'{flag}{RELEASE_RATE}'] = f'{(release_count / release_count * 100):.2f}'
			reached_count = sum((df['位置'] == slot) & ((df['位姿达标'] == 1) | (df['位姿是否达标'] == 1)))
			analysis_result[f'{flag}{POSTURE_RIGHT_COUNT}'] = reached_count
			analysis_result[f'{flag}{POSTURE_RIGHT_RATE}'] = f'{(reached_count / release_count * 100):.2f}'

			hit_wheel_stop_count = 0
			if(not df['问题归类'].isna().all()):	
				hit_wheel_stop_count = sum((df['位置'] == slot) & (df['问题归类'].str.count('撞轮挡')))
			analysis_result[f'{flag}{HIT_WHEELSTOP_COUNT}'] = hit_wheel_stop_count
			analysis_result[f'{flag}{NOT_HIT_WHEELSTOP_RATE}'] = f'{(100 - hit_wheel_stop_count / release_count * 100):.2f}'
			idx += 1
	return analysis_result

def transform_date(date_str):
	return datetime.strptime(date_str, "%Y年%m月%d日").strftime('%Y-%m-%d')

def vehicle_type_sum(df):
	vehicle_type_df = pd.DataFrame()
	vehicle_type_df = df.iloc[0,:7]
	vehicle_type_df[PARKING_COUNT] = sum(df[PARKING_COUNT])

	vehicle_type_df[RELEASE_COUNT] = sum(df[RELEASE_COUNT])
	release_rate = vehicle_type_df[RELEASE_COUNT]/vehicle_type_df[PARKING_COUNT] * 100
	vehicle_type_df[RELEASE_RATE] = f'{release_rate:.2f}'

	vehicle_type_df[POSTURE_RIGHT_COUNT] = sum(df[POSTURE_RIGHT_COUNT])
	posture_right_rate = vehicle_type_df[POSTURE_RIGHT_COUNT] / vehicle_type_df[RELEASE_COUNT] * 100
	vehicle_type_df[POSTURE_RIGHT_RATE] = f'{posture_right_rate:.2f}'

	vehicle_type_df[HIT_WHEELSTOP_COUNT] = sum(df[HIT_WHEELSTOP_COUNT])
	not_hit_wheel_stop_rate = 100 - vehicle_type_df[HIT_WHEELSTOP_COUNT]/vehicle_type_df[RELEASE_COUNT] * 100
	vehicle_type_df[NOT_HIT_WHEELSTOP_RATE] = f'{not_hit_wheel_stop_rate:.2f}'
	return vehicle_type_df

def slot_sum(df):
	columns=[
			DATE,
			VERSION,
			BAG,
			CITY,
			DATA_LINK,
			SLOT,
		]
	
	for add in ['ADD', 'ADD2']:
		for item in DATA_GROUP:
			columns.append(f'{add}{item}')
	slot_df = pd.DataFrame(columns=columns)

	df = df.reset_index(drop=True)
	slot_df.loc[len(slot_df)] = [None] * len(slot_df.columns)  # 添加一行空值
	slot_df[DATE] = df[DATE][0]
	slot_df[VERSION] = df[VERSION][0]
	slot_df[BAG] = df[BAG][0]
	slot_df[CITY] = df[CITY][0]
	slot_df[DATA_LINK] = df[DATA_LINK][0]
	slot_df[SLOT] = df[SLOT][0]


	vehicle_type_df = df.groupby(V_TYPE).apply(vehicle_type_sum)
	vehicle_type_df = vehicle_type_df.reset_index(drop=True)	
	ADD_data = vehicle_type_df[vehicle_type_df[V_TYPE] == 'ADD']
	ADD_data = ADD_data.reset_index(drop=True)
	slot_df.iloc[:,6:13] = ADD_data.iloc[:,7:14]

	ADD2_data = vehicle_type_df[vehicle_type_df[V_TYPE] == 'ADD2']
	ADD2_data = ADD2_data.reset_index(drop=True)
	slot_df.iloc[:,13:20] = ADD2_data.iloc[:,7:14]
		
	return slot_df

def reorgnize_df_by_slot(df):
	reorgnized_df = pd.DataFrame()
	for idx, row in df.iterrows():
		slot_count = 0
		flag = number_to_letter(slot_count)
		column_name = f'{SLOT}{flag}'
		slot_name = row[column_name]
		while(not pd.isnull(slot_name)):
			new_df = row.iloc[:6]
			new_df[SLOT] = slot_name
			for item in DATA_GROUP:
				new_df[item] = row[f'{flag}{item}']
			slot_count += 1
			flag = number_to_letter(slot_count)
			column_name = f'{SLOT}{flag}'
			if(column_name not in df.columns):
				break
			slot_name = row[column_name]
			reorgnized_df = reorgnized_df.append(new_df, ignore_index=True)
	
	return reorgnized_df

def create_html_file(file_name, pages):
    with open(file_name, 'w', encoding='utf-8') as f:
        f.write('<!DOCTYPE html>\n')
        f.write('<html>\n')
        f.write('<head><title>Simple Page Selector</title></head>\n')
        f.write('<body>\n')
        f.write('<select id="page-selector" onchange="changePage(this.value)">\n')
        for index, page in enumerate(pages):
            f.write(f'  <option value="{index}">{page["city"]}</option>\n')
        f.write('</select>\n')
        f.write('<div id="page-content">\n')
        for page in pages:
            f.write(f'  <div class="page-content" style="display: none;">{page["content"]}</div>\n')
        f.write('</div>\n')
        f.write('<script>\n')
        f.write('  function changePage(index) {\n')
        f.write('    var pages = document.getElementsByClassName("page-content");\n')
        f.write('    for (var i = 0; i < pages.length; i++) {\n')
        f.write('      pages[i].style.display = "none";\n')
        f.write('    }\n')
        f.write('    pages[index].style.display = "block";\n')
        f.write('  }\n')
        f.write('</script>\n')
        f.write('</body>\n')
        f.write('</html>')

def plot_by_slot(df):
	bar_width = 0.1
	vertical_spacing = 0.04		
	subplot_height = 100
	width = 1800
	
	color_dict = {
		f'ADD{POSTURE_RIGHT_RATE}':'green',  
		f'ADD2{POSTURE_RIGHT_RATE}':'orange',  
		f'ADD{NOT_HIT_WHEELSTOP_RATE}':'red',
		f'ADD2{NOT_HIT_WHEELSTOP_RATE}':'pink'
		}
	citys = df[CITY].unique()
	pages = []
	for city in citys:
		city_result = {}
		city_df = df[df[CITY]==city]
		city_dir = os.path.join('original_data_by_slot', city)
		city_dir_grouped = os.path.join('original_data_by_slot_grouped', city)
		if(os.path.exists(city_dir)):
			shutil.rmtree(city_dir)
		os.makedirs(city_dir)
		if(os.path.exists(city_dir_grouped)):
			shutil.rmtree(city_dir_grouped)
		os.makedirs(city_dir_grouped)
		slots = city_df[SLOT].unique()
		slot_num = len(slots)
		
		fig = make_subplots(rows= slot_num,
												cols=1,
												shared_xaxes=True, 
												subplot_titles=slots,
												# vertical_spacing=vertical_spacing
												)
		slot_id = 1
		for slot in slots:
			slot_df = city_df[city_df[SLOT]==slot]
			output_name = os.path.join(city_dir,f'{slot}.csv')
			slot_df.to_csv(output_name)
			slot_df_group = slot_df.groupby(BAG).apply(slot_sum)
			output_name_grouped = os.path.join(city_dir_grouped,f'{slot}.csv')
			slot_df_group.to_csv(output_name_grouped)
			slot_df_group[BAG] = slot_df_group[BAG].astype(str)
			slot_df_group.iloc[:,7:20] = slot_df_group.iloc[:,7:20].astype(float)

			for i, column in enumerate([f'ADD{POSTURE_RIGHT_RATE}',  f'ADD2{POSTURE_RIGHT_RATE}',  f'ADD{NOT_HIT_WHEELSTOP_RATE}',f'ADD2{NOT_HIT_WHEELSTOP_RATE}']):
				if(slot_df_group[column][0]==None):
					continue
				showlegend = False
				if(slot_id == 1):
					showlegend= True
				fig.add_trace(
					go.Bar(
						x = slot_df_group[BAG],
						y = slot_df_group[column],
						name = column,
						legendgroup = column,
						showlegend=showlegend,
						width = bar_width,
						marker_color=color_dict[column],
					),
					row=slot_id, 
					col=1
				)
			xaxis_name = f'xaxis{slot_id}'
			yaxis_name = f'yaxis{slot_id}'
			if(slot_id==1):
				xaxis_name = 'xaxis'
				yaxis_name = 'yaxis'
			fig["layout"][xaxis_name]["showticklabels"] = True
			fig["layout"][yaxis_name]["range"] = [0,100]
			slot_id += 1

		fig.update_layout(
				title_text='机械库位',
				title_font=dict(size=50, family='Arial, sans-serif', color='black'),
				title_x=0.5,
				
				# xaxis_title='bag ID',
				barmode='group',  # 设置为分组模式
				bargap=0.5,
				height=subplot_height * 1.1 * slot_num,
				width=width,
		)
		city_result['city'] = f'{city}({slot_num})'
		city_result['content'] = fig.to_html()
		pages.append(city_result)

	create_html_file('机械库位停车场测试情况.html', pages)

def main(args):
	folder_path = 'original_data'
	result_df = pd.DataFrame()

	for filename in os.listdir(folder_path):
		if filename.endswith('.csv'):
			file_path = os.path.join(folder_path, filename)
			df = pd.read_csv(file_path)
			analysis_result = analyze_data(df,filename)
			result_df = result_df.append(analysis_result, ignore_index=True)
			
	result_df[DATE] = result_df[DATE].apply(transform_date)
	output_file_by_date = 'analyze_result_by_date.csv'
	result_df.to_csv(output_file_by_date, index=False, encoding='utf-8-sig')

	print('****************************************************')
	output_file_by_slot = 'analyze_result_by_slot.csv'
	reorgnized_df = reorgnize_df_by_slot(result_df)
	plot_by_slot(reorgnized_df)
	reorgnized_df.to_csv(output_file_by_slot, index=False, encoding='utf-8-sig')

if __name__ == '__main__':
    parser = argparse.ArgumentParser('Report Generator')
    args = parser.parse_args()
    
    main(args)