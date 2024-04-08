#!/usr/bin/env python3
from ctypes import sizeof
from locale import DAY_1
import psycopg2
import psycopg2.extras
from clickhouse_sqlalchemy import make_session
from sqlalchemy import create_engine

import datetime
import pandas as pd
import os

TIME = 0
SPEED = 1
GEAR = 2
PEDAL_OUTPUT = 3
ACC = 4
ENGINR_RPM = 5
PITCH = 6
WEIGHT = 7
BRAKE = 8
FUEL_RATE = 9
RETARDER = 10
BRAKE_SPEED = 11
PEDAL_SPEED = 12
BSFC = 13

TARGET_SIGNAL_LIST = ['time', 'speed', 'gear', 'pedal_output', 'acc', 'engine_rpm', 'pitch',
                     'weight', 'brake','fuel_rate','retarder', 'brake_speed', 'pedal_speed', 'bsfc']

CH_sql = '''
    select
        ts,
        JSONExtractFloat(vehicle_dbw_reports, 'steeringReport', 'speed') as speed,
        JSONExtractInt(vehicle_dbw_reports, 'gearReport', 'cmd') as gear_cmd,
        JSONExtractFloat(vehicle_dbw_reports, 'throttleReport', 'pedalOutput') as pedal_output,
        JSONExtractFloat(vehicle_control_cmd, 'debugCmd', 'aReport') as acc_control,
        JSONExtractFloat(vehicle_dbw_reports, 'throttleInfoReport', 'engineRpm') as engine_rpm,
        JSONExtractFloat(vehicle_status, 'pitch') as road_pitch,
        JSONExtractFloat(vehicle_control_cmd, 'debugCmd', 'totalWeight') as weight,
        JSONExtractFloat(vehicle_dbw_reports, 'brakeReport', 'pedalInput') as brake_pedal,
        JSONExtractFloat(vehicle_dbw_reports, 'fuelEconomy', 'engineFuelRate') as fuel_rate,
        JSONExtractFloat(vehicle_dbw_reports, 'retarderMsg', 'actualRetarderPercentTorque') as retarder_torque
        
    from bag_messages bm
    WHERE vehicle = '{0}' and ts >= '{1}' AND ts <= '{2}' AND bag_source = 'offline'
    AND (vehicle_dbw_reports is not NULL or vehicle_control_cmd is not NULL or vehicle_status is not NULL)
    order by ts
'''
PRE_TIME_OFFSET = 0.2
POST_TIME_OFFSET = 0.005
class DataParser:
    def __init__(self, target_file, mode):
        self._target_file = target_file
        self._last_data_list = len(TARGET_SIGNAL_LIST) * [0]
        self._start_row = 1
        self._mode = mode
        init_df = pd.DataFrame(columns = TARGET_SIGNAL_LIST)
        init_df.to_excel(self._target_file, sheet_name='Sheet1', index=False)

    def set_data(self,data_list,data_query,index):
        i = 1
        while(data_query[-i][index]== None and i < len(data_query)):
            i += 1
        data_list[index] = data_query[-i][index] if data_query[-i][index] != None else self._last_data_list[index]

    def set_speed_data(self,data_list,data_query,sourceindex, index):
        left = 0
        right = len(data_query)-1
        left_found = False
        right_found = False
        while(left < right and (not left_found or not right_found)):
            if(data_query[left][sourceindex] == None or data_query[left][TIME]==None):
                left += 1
            else:
                left_found = True
            if(data_query[right][sourceindex] == None or data_query[right][TIME]==None):
                right -= 1
            else:
                right_found = True
        if(left < right):
            left_time = data_query[left][TIME].strftime('%Y%m%d%H%M%S%f')
            right_time = data_query[right][TIME].strftime('%Y%m%d%H%M%S%f')
            delta_t = (int(right_time) - int(left_time))/100000 # unit:s
            data_list[index] = (data_query[right][sourceindex] - data_query[left][sourceindex])/(delta_t+0.01)
        else:
            data_list[index] = self._last_data_list[index]

    def compute_bsfc(self,data_list):
        data_list[BSFC] = data_list[FUEL_RATE] / (data_list[PEDAL_OUTPUT],data_list[ENGINR_RPM])


    def archive_data(self, data_query):
        df = pd.DataFrame(columns = TARGET_SIGNAL_LIST)
        data_list = len(TARGET_SIGNAL_LIST)*[0]
        

        self.set_data(data_list,data_query,TIME)
        self.set_data(data_list,data_query,SPEED)
        self.set_data(data_list,data_query,GEAR)
        self.set_data(data_list,data_query,PEDAL_OUTPUT)
        self.set_data(data_list,data_query,ENGINR_RPM)
        self.set_data(data_list,data_query,PITCH)
        self.set_data(data_list,data_query,WEIGHT)
        self.set_data(data_list,data_query,BRAKE)
        self.set_data(data_list,data_query,FUEL_RATE)
        self.set_data(data_list,data_query,RETARDER)
        self.set_speed_data(data_list,data_query,SPEED,ACC)
        self.set_speed_data(data_list,data_query,BRAKE,BRAKE_SPEED)
        self.set_speed_data(data_list,data_query,PEDAL_OUTPUT,PEDAL_SPEED)

        df.loc[len(df)] = data_list

        df.to_excel(self._target_file, sheet_name='Sheet1', startrow=self._start_row, index=False, header=None)
        self._start_row += len(df)

    def close(self):
        self._target_file.close()                      


    
        

 
def main():
    CH_conf = {
        "user": "plus_viewer",
        "password": "ex4u1balSAeR68uC",
        "server_host": "clickhouse-proxy-cn.plusai.co",
        "port": "9090",
        "db": "bagdb"
    }

    connection = 'clickhouse://{user}:{password}@{server_host}:{port}/{db}'.format(**CH_conf)
    engine = create_engine(connection, pool_size=100, pool_recycle=3600, pool_timeout=20)
    session = make_session(engine)


    roadtest_conn = \
        psycopg2.connect("dbname=roadtest user=roadtest host=192.168.10.18 password=systemtest options='-c statement_timeout=600000'")

    roadtest_cur = roadtest_conn.cursor(cursor_factory=psycopg2.extras.RealDictCursor)

    sql = '''
        SELECT *
        FROM public.shifting_gear_points
        WHERE gear_type = 'cmd' AND vehicle = 'j7-l4e-LFWSRXSJ8M1F50115'
        limit 300000
    '''
    roadtest_cur.execute(sql)
    res = roadtest_cur.fetchall()
    print(len(res))
    target_vehicles = ['j7-l4e-c0004_LFWSRXSJ7M1F45293','j7-l4e-LFWSRXSJ6M1F50503','j7-l4e-LFWSRXSJ8M1F50115']

    excel_writer_dict = {}
    for fe_mode in ['DP','DE']:
        vehicle_dict = {}
        for target_vehicle_name in target_vehicles: 
            filepath = os.path.join(os.environ['HOME'],'data/gear_shift', fe_mode, target_vehicle_name)
            if os.path.exists(filepath):
                print("the dir existed")
            else:
                os.makedirs(filepath) 
            auto_upshift_path = os.path.join(filepath,'auto_upshift.xlsx')
            auto_downshift_path = os.path.join(filepath,'auto_downshift.xlsx')
            manual_upshift_path = os.path.join(filepath,'manual_upshift.xlsx')
            manual_downshift_path = os.path.join(filepath,'manual_downshift.xlsx')
            
            au_writer = pd.ExcelWriter(auto_upshift_path)
            ad_writer = pd.ExcelWriter(auto_downshift_path)
            mu_writer = pd.ExcelWriter(manual_upshift_path)
            md_writer = pd.ExcelWriter(manual_downshift_path)
            au_data_parser = DataParser(au_writer,'auto_upshift')
            ad_data_parser = DataParser(ad_writer,'auto_downshift')
            mu_data_parser = DataParser(mu_writer,'manual_upshift')
            md_data_parser = DataParser(md_writer,'manual_downshift')
            vehicle_dict[target_vehicle_name] = {
                                                    'auto_upshift':au_data_parser,
                                                    'auto_downshift':ad_data_parser,
                                                    'manual_upshift':mu_data_parser,
                                                    'manual_downshift':md_data_parser
                                                }
        excel_writer_dict[fe_mode] = vehicle_dict

    print(excel_writer_dict)
    i = 0
    for shift_point in res:
        vehicle_name = shift_point['vehicle']
        if(shift_point['fe_mode'] == 0):
            fe_mode = 'DP'
        elif(shift_point['fe_mode'] == 1):
            fe_mode = 'DE'
        else:
            fe_mode = 'DE'
        
        shift_ts = shift_point['shift_ts']
        date_time = datetime.datetime.fromtimestamp(shift_ts) - datetime.timedelta(hours=8) # ts is UTC time in Clickhouse database

        pre_time_offset = datetime.timedelta(seconds = PRE_TIME_OFFSET)
        post_time_offset = datetime.timedelta(seconds = POST_TIME_OFFSET)
        pre_time = (date_time - pre_time_offset).strftime("%Y-%m-%d %H:%M:%S.%f")
        post_time = (date_time + post_time_offset).strftime("%Y-%m-%d %H:%M:%S.%f")
    
        cursor = session.execute(CH_sql.format(vehicle_name, pre_time, post_time))
        data_query = cursor.fetchall()

        shift_type = shift_point['shift_type'].split('-')
        if(shift_type[0] > shift_type[1]):
            if(shift_point['mode'] == 1):
                # print('auto_downshift')
                excel_writer_dict[fe_mode][vehicle_name]['auto_downshift'].archive_data(data_query)
            else:
                # print('manual_downshift')  
                excel_writer_dict[fe_mode][vehicle_name]['manual_downshift'].archive_data(data_query)
        else:
            if(shift_point['mode'] == 1):
                # print('auto_upshift')
                excel_writer_dict[fe_mode][vehicle_name]['auto_upshift'].archive_data(data_query)
            else: 
                # print('manual_upshift')
                excel_writer_dict[fe_mode][vehicle_name]['manual_upshift'].archive_data(data_query)
        # print(shift_point)
        # print("======================")
        # print(data_query)
        # print("**********************************************")   
        i += 1
        print("i = ",i)                             
                
    cursor.close()
    session.close()
    for fe_mode in ['DP','DE']:
        for target_vehicle_name in target_vehicles:
            for item in excel_writer_dict[fe_mode][target_vehicle_name].items():
                item[1].close()
    print("===========archive data successfully===========")       
if __name__ == '__main__':
    main()