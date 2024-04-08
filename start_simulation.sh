#!/bin/bash
RunPlusMapScenarioSimulation(){
	${lwd}/simulator/scenario/tools/run_scenario_simulation \
		--vehicle-name ${vehicle_name} \
		--scenario-file $1 \
		--start-ros \
		--build \
		--output-dir ${output_dir} \
		--vehicle-model-node dynamics_model \
		--latency-verification-mode warn \
		--controller-mode control_v2 \
		--ipc-pubsub-modes shm_bus,ros \
		--bag-recorder rosbag \
		--no-simulator-ui \
		--vehicle-mode fuel_efficiency \
		--show-trailer \
		--map-type plusmap \
		--region-map-type MAP \
		--planning-map-mode PLUSMAP \
		--enable-lane-path-simulator \
		--timeout-after 500 \
		--planner-visualizer-mode record-video \
		--use-unified-simulator \
		--run-pnc-analyzer
}
RunNaviLaneScenarioSimulation(){
	${lwd}/simulator/scenario/tools/run_scenario_simulation \
		--vehicle-name ${vehicle_name} \
		--scenario-file $1 \
		--start-ros \
		--build \
		--output-dir ${output_dir} \
		--vehicle-model-node dynamics_model \
		--latency-verification-mode warn \
		--controller-mode control_v2 \
		--ipc-pubsub-modes shm_bus,ros \
		--bag-recorder rosbag \
		--no-simulator-ui \
		--vehicle-mode fuel_efficiency \
		--show-trailer \
		--map-type navinfo \
		--region-map-type LANE \
		--planning-map-mode NAVIGATION \
		--localization-mapping-path $2 \
		--ros-simulator-use-lane-detection \
		--timeout-after 500 \
		--planner-visualizer-mode record-video \
		--use-unified-simulator \
		--run-pnc-analyzer
}

RunNaviMapScenarioSimulation(){
	${lwd}/simulator/scenario/tools/run_scenario_simulation \
		--vehicle-name ${vehicle_name} \
		--scenario-file $1 \
		--start-ros \
		--build \
		--output-dir ${output_dir} \
		--vehicle-model-node dynamics_model \
		--latency-verification-mode warn \
		--controller-mode control_v2 \
		--ipc-pubsub-modes shm_bus,ros \
		--bag-recorder rosbag \
		--no-simulator-ui \
		--vehicle-mode fuel_efficiency \
		--show-trailer \
		--map-type plusmap \
		--region-map-type LANE \
		--planning-map-mode NAVIGATION \
		--enable-lane-path-simulator \
		--timeout-after 500 \
		--planner-visualizer-mode record-video \
		--use-unified-simulator
}

RunBenchmarkBagUnifiedSimulation(){
	${lwd}/simulator/scenario/tools/run_scenario_simulation \
		--vehicle-name ${vehicle_name} \
		--test-file $1 \
		--start-ros \
		--build \
		--output-dir ${output_dir} \
		--vehicle-model-node dynamics_model \
		--latency-verification-mode warn \
		--controller-mode control_v2 \
		--ipc-pubsub-modes shm_bus,ros \
		--bag-recorder rosbag \
		--no-simulator-ui \
		--vehicle-mode fuel_efficiency \
		--show-trailer \
		--map-type navinfo \
		--region-map-type LANE \
		--planning-map-mode NAVIGATION \
		--ros-simulator-use-lane-detection \
		--timeout-after 500 \
		--planner-visualizer-mode record-video \
		--use-unified-simulator \
		--run-pnc-analyzer
}
RunBenchmarkBagManualUnifiedSimulation(){
	${lwd}/simulator/scenario/tools/run_scenario_simulation \
		--vehicle-name ${vehicle_name} \
		--test-file $1 \
		--start-ros \
		--build \
		--output-dir ${output_dir} \
		--vehicle-model-node dynamics_model \
		--latency-verification-mode warn \
		--controller-mode control_v2 \
		--ipc-pubsub-modes shm_bus,ros \
		--bag-recorder rosbag \
		--no-simulator-ui \
		--vehicle-mode fuel_efficiency \
		--show-trailer \
		--map-type navinfo \
		--region-map-type LANE \
		--planning-map-mode NAVIGATION \
		--ros-simulator-use-lane-detection \
		--timeout-after 500 \
		--manual-set-cruise-speed \
		--planner-visualizer-mode record-video \
		--use-unified-simulator \
		--run-pnc-analyzer
}
RunBenchmarkScenarioUnifiedSimulation(){
	${lwd}/simulator/scenario/tools/run_scenario_simulation \
		--vehicle-name ${vehicle_name} \
		--test-file $1 \
		--start-ros \
		--build \
		--output-dir ${output_dir} \
		--vehicle-model-node dynamics_model \
		--latency-verification-mode warn \
		--controller-mode control_v2 \
		--ipc-pubsub-modes shm_bus,ros \
		--bag-recorder rosbag \
		--no-simulator-ui \
		--vehicle-mode fuel_efficiency \
		--show-trailer \
		--map-type plusmap \
		--region-map-type LANE \
		--planning-map-mode NAVIGATION \
		--manual-set-cruise-speed \
		--enable-lane-path-simulator \
		--timeout-after 500 \
		--planner-visualizer-mode record-video \
		--use-unified-simulator \
		--run-pnc-analyzer
}
RunBenchmarkScenarioManualUnifiedSimulation(){
	${lwd}/simulator/scenario/tools/run_scenario_simulation \
		--vehicle-name ${vehicle_name}-manual \
		--test-file $1 \
		--start-ros \
		--build \
		--output-dir ${output_dir} \
		--vehicle-model-node dynamics_model \
		--latency-verification-mode warn \
		--controller-mode control_v2 \
		--ipc-pubsub-modes shm_bus,ros \
		--bag-recorder rosbag \
		--no-simulator-ui \
		--vehicle-mode fuel_efficiency \
		--show-trailer \
		--map-type plusmap \
		--region-map-type LANE \
		--planning-map-mode NAVIGATION \	
		--enable-lane-path-simulator \
		--timeout-after 500 \
		--manual-set-cruise-speed \
		--planner-visualizer-mode record-video \
		--use-unified-simulator \
		--run-pnc-analyzer
}

CopyOutput(){
	scenario_file=$1
	param_tunning=$2
	scenario_file_name=${scenario_file##*/}
	scenario_file_name_without_postfix=${scenario_file_name%.*}

	target_dir=${pwd}/report/simulation_report
    echo ${scenario_file_name_without_postfix}
	if [ ${param_tunning} == 'default' ];then
		target_report_filepath=${target_dir}/${time}/${scenario_file_name_without_postfix}	
	else
		target_report_filepath=${target_dir}/${time}/${vehicle}/${param_tunning}/${scenario_file_name_without_postfix}
	fi
	echo ${target_report_filepath}
	if [ ! -d ${target_report_filepath} ]; then
		mkdir -p ${target_report_filepath}
	fi
	cp -r ${output_dir} ${target_report_filepath}

	bag_dir=${output_dir}/bag
	python ./tools/report_generator.py ${bag_dir} --output-dir ${target_report_filepath}

	echo "==============finish custom report ${scenario_file_name_without_postfix}==================="

	# cd ${lwd}/tools/fuel_economy/engine_operation_points_analysis
	# for file in $(ls ${bag_dir})
	# do 
    #     bag_fullpath=${bag_dir}/${file}
    #     python trip_detail_analysis.py example/trip_info_config.yaml ${target_report_filepath}  --task 1 --from_bag --bag_path ${bag_fullpath}
	# done

	# echo "==============finish generate fuel data report==================="
}

# mode = single: simulation for one scenario_file
# mode = batch: simulatoon for scenario_files in one folder
mode=$1
single_type=${2:-navi_map}
vehicle=${3:-pdb}
param_tunning=${4:-default}
current_time=${5:-xxxx}
vehicle_name=${vehicle}-l4e-sim
if [ ${current_time} == 'xxxx' ];then
	time=$(date "+%Y%m%d%H%M%S")	
else
	time=${current_time}
fi

pwd=$(pwd -P)
lwd=$(dirname $pwd)
export DRIVE_ROOT=../drive/opt/relwithdebinfo	
export GLOG_v=4
export GLOG_minloglevel=0
output_dir=${lwd}/simulator/scenario/simulation_output
benchmark_dir=${lwd}/simulator/scenario/scenario_test/benchmark/CN/L4E
sudo rm -f /dev/shm/*
rosparam set use_sim_time false

if [ ${mode} == 'batch_folder' ];then
	scenario_file_dir=${lwd}/simulator/scenario/scenario_test/data/CN/L4E/coasting
	for file in $(ls ${scenario_file_dir})
	do 
		postfix=${file##*.}
		if [ ${postfix} != 'prototxt' ];then
			continue
		fi
		sudo rm -f /dev/shm/*
		rosparam set use_sim_time false
		scenario_file=${scenario_file_dir}/${file}
		RunNaviMapScenarioSimulation ${scenario_file}
		CopyOutput ${scenario_file} ${param_tunning}
	done	
elif [ ${mode} == 'batch_list' ];then
	if [ ${single_type} == 'navi_map' ];then
        while read scenario_file
        do
            echo ${scenario_file}
            RunNaviMapScenarioSimulation ${scenario_file}
            CopyOutput ${scenario_file} ${param_tunning}     
        done < ./scenario_list.lst
	elif [ ${single_type} == 'navi_lane' ];then
        scenario_list_dir=${pwd}/scenarios/scenario_list/latest_scenario_list
        for sub_dir in $(ls ${scenario_list_dir})
        do
            sudo rm -f /dev/shm/*
            rosparam set use_sim_time false
            scenario_file=${scenario_list_dir}/${sub_dir}/${sub_dir}.protobin
            mapping_file=${scenario_list_dir}/${sub_dir}/${sub_dir}_mapping.prototxt
            RunNaviLaneScenarioSimulation ${scenario_file} ${mapping_file}
            CopyOutput ${scenario_file} ${param_tunning}
            sleep 30s
        done
    fi
elif [ ${mode} == 'single' ];then
	if [ ${single_type} == 'plusmap' ];then
		scenario_file=${lwd}/simulator/scenario/scenario_test/data/CN/L4E/coasting/scenario_coasting_02_01.prototxt
		RunPlusMapScenarioSimulation ${scenario_file}
	elif [ ${single_type} == 'navi_lane' ];then
		single_scenario_dir=${pwd}/scenarios/single_scenario/latest_scenario
		for sub_dir in $(ls ${single_scenario_dir})
		do
			sudo rm -f /dev/shm/*
			rosparam set use_sim_time false
			scenario_file=${single_scenario_dir}/${sub_dir}/${sub_dir}.protobin
			mapping_file=${single_scenario_dir}/${sub_dir}/${sub_dir}_mapping.prototxt
			RunNaviLaneScenarioSimulation ${scenario_file} ${mapping_file}
		done
	elif [ ${single_type} == 'navi_map' ];then
		scenario_file=${lwd}/simulator/scenario/scenario_test/data/CN/L4E/coasting/scenario_coasting_02_05.prototxt
		# scenario_file=${lwd}/simulator/scenario/scenario_test/data/suzhou_circle/suzhou_circle/suzhou_circle.prototxt
		# scenario_file=${lwd}/simulator/scenario/scenario_test/data/g50/g50/g50.prototxt
		# scenario_file=${lwd}/simulator/scenario/scenario_test/data/CN/L4E/fe/fuel_consumption_01_04.prototxt
		RunNaviMapScenarioSimulation ${scenario_file}
	elif [ ${single_type} == 'benchmark_bag_manual_unified' ];then
		scenario_file=${benchmark_dir}/merge/benchmark_merge_20220106T082027_j7-l4e-LFWSRXSJ0M1F48987_1_184to218.prototxt
		RunBenchmarkBagManualUnifiedSimulation ${scenario_file}	
	elif [ ${single_type} == 'benchmark_bag_unified' ];then
		scenario_file=${benchmark_dir}/merge/benchmark_merge_20220106T103531_j7-l4e-LFWSRXSJ7M1F48985_9_1926to1960.prototxt
		scenario_file=${benchmark_dir}/suggest_lane_change/benchmark_suggest_lane_change_20220629T021501_j7-l4e-LFWSRXSJXM1F50505_0_885to950.prototxt
		scenario_file=${benchmark_dir}/fe/benchmark_fe_20221205T095000_j7-l4e-LFWSRXSJ7M1F48985_2_1001to1041.prototxt
		scenario_file=${benchmark_dir}/fe/benchmark_fe_20221117T134511_j7-l4e-LFWSRXSJ7M1F48985_4_1to601.prototxt
		# scenario_file=${benchmark_dir}/fe/benchmark_fe_20211026T184250_j7-l4e-c0002_LFWSRXSJ4M1F44912_3_680to720.prototxt
		# scenario_file=${benchmark_dir}/nudge/benchmark_nudge_20220918T002631_j7-l4e-LFWSRXSJ5M1F50508_15_420to445.prototxt
		# scenario_file=${benchmark_dir}/cut_in_out/benchmark_cutinout_20210902T024101_j7-l4e-00013_0_1991to2001.prototxt
		# scenario_file=${benchmark_dir}/cut_in_out/benchmark_cutinout_20210917T143800_j7-l4e-b0005_0_1860to1870.prototxt
		# scenario_file=${benchmark_dir}/cut_in_out/benchmark_cutinout_20211016T024539_j7-l4e-LFWSRXSJ5M1F45065_3_621to636.prototxt
		scenario_file=${benchmark_dir}/fe/benchmark_fe_20221117T134511_j7-l4e-LFWSRXSJ7M1F48985_4_1to601.prototxt
		RunBenchmarkBagUnifiedSimulation ${scenario_file}	
	elif [ ${single_type} == 'benchmark_scenario_manual_unified' ];then
		scenario_file=${benchmark_dir}/aeb/CRUISING_LEAD/benchmark_scenario_aeb_manual_static_vehicle_40kph.prototxt
		RunBenchmarkScenarioManualUnifiedSimulation ${scenario_file}
	elif [ ${single_type} == 'benchmark_scenario_unified' ];then
		scenario_file=${benchmark_dir}/alc/benchmark_scenario_change_04_03.prototxt # pass
		scenario_file=${benchmark_dir}/coasting/benchmark_scenario_coasting_01_01.prototxt
		scenario_file=${benchmark_dir}/fe/benchmark_fuel_consumption_01_04.prototxt
		RunBenchmarkScenarioUnifiedSimulation ${scenario_file}
	fi

	CopyOutput ${scenario_file} ${param_tunning}
else
	echo "[ERROR]: mode must be 'batch' or 'single'"
fi
