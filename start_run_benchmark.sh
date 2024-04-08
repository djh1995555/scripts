# !/bin/bash

sudo rm -f /dev/shm/*
pwd=$(pwd -P)
lwd=$(dirname $pwd)
time=$(date "+%Y%m%d%H%M%S")
export drive_root=../drive/opt/relwithdebinfo
vehicle=${1:-j7}
vehicle_name=${vehicle}-l4e-sim

current_time_filepath=${pwd}/report/benchmarks/${time}
if [ ! -d ${current_time_filepath} ]; then
  mkdir -p ${current_time_filepath}
fi

RunBenchmarkBagSimulation(){
${lwd}/simulator/scenario/tools/run_scenario_simulations \
  --latency-verification-mode skip \
  --num-runs 1 \
  --max-retries 0 \
  --output-dir ${current_time_filepath}/benchmark_bag \
  --vehicle-name $1 \
  --controller-mode control_v2 \
  --vehicle-model-node dynamics_model \
  --planning-map-mode NAVIGATION \
  --vehicle-mode fuel_efficiency \
  --region-map-type LANE \
  --ros-simulator-use-lane-detection \
  --map-type navinfo \
  --planner-visualizer-mode on \
  --run-pnc-analyzer \
  --build \
  --use-unified-simulator \
  --timeout-after 500 \
  --base-run-name master_0608 ${lwd}/simulator/scenario/jenkins/benchmarks_cn_fe_scenario_with_traffic_stream.lst
  # ${lwd}/simulator/scenario/jenkins/benchmarks_cn_nudge.lst
  # ${lwd}/simulator/scenario/jenkins/benchmarks_cn_suggest_lane_change.lst
  # ${lwd}/simulator/scenario/jenkins/benchmarks_cn_cutinout.lst
  # ${lwd}/simulator/scenario/jenkins/benchmarks_cn_stopngo.lst
  # ${lwd}/simulator/scenario/jenkins/benchmarks_cn_fe_scenario_with_traffic_stream.lst
} 

RunBenchmarkBagSetCruiseSpeedSimulation(){
${lwd}/simulator/scenario/tools/run_scenario_simulations \
  --latency-verification-mode skip \
  --num-runs 1 \
  --max-retries 0 \
  --output-dir ${current_time_filepath}/benchmark_bag_cruise_speed \
  --vehicle-name $1 \
  --controller-mode control_v2 \
  --vehicle-model-node dynamics_model \
  --planning-map-mode NAVIGATION \
  --vehicle-mode fuel_efficiency \
  --region-map-type LANE \
  --ros-simulator-use-lane-detection \
  --map-type navinfo \
  --planner-visualizer-mode record-video \
  --run-pnc-analyzer \
  --manual-set-cruise-speed \
  --use-unified-simulator \
  --build \
  --timeout-after 500 \
  --base-run-name master_0608 ${lwd}/simulator/scenario/jenkins/benchmarks_cn_merge.lst
  # 
} 

RunBenchmarkScenarioSetCruiseSpeedSimulation(){
${lwd}/simulator/scenario/tools/run_scenario_simulations \
  --latency-verification-mode skip \
  --num-runs 1 \
  --max-retries 0 \
  --output-dir ${current_time_filepath}/benchmark_scenario \
  --vehicle-name $1 \
  --controller-mode control_v2 \
  --vehicle-model-node dynamics_model \
  --planning-map-mode NAVIGATION \
  --vehicle-mode fuel_efficiency \
  --region-map-type LANE \
  --enable-lane-path-simulator \
  --map-type plusmap \
  --planner-visualizer-mode record-video \
  --run-pnc-analyzer \
  --manual-set-cruise-speed \
  --use-unified-simulator \
  --build \
  --timeout-after 500 \
  --base-run-name master_0608 ${lwd}/simulator/scenario/jenkins/benchmarks_cn_fe_scenario.lst
  # ${lwd}/simulator/scenario/jenkins/benchmarks_control_1.lst
  # ${lwd}/simulator/scenario/jenkins/benchmarks_cn_cruising.lst
  # ${lwd}/simulator/scenario/jenkins/benchmarks_cn_fe_scenario.lst
  # ${lwd}/simulator/scenario/jenkins/benchmarks_cn_alc.lst
} 

RunBenchmarkScenarioSetCruiseSpeedManualSimulation(){
${lwd}/simulator/scenario/tools/run_scenario_simulations \
  --latency-verification-mode skip \
  --num-runs 1 \
  --max-retries 0 \
  --output-dir ${current_time_filepath}/benchmark_scenario_manual \
  --vehicle-name $1 \
  --controller-mode control_v2 \
  --vehicle-model-node dynamics_model \
  --planning-map-mode NAVIGATION \
  --vehicle-mode fuel_efficiency \
  --region-map-type LANE \
  --enable-lane-path-simulator \
  --map-type plusmap \
  --planner-visualizer-mode record-video \
  --run-pnc-analyzer \
  --manual-set-cruise-speed \
  --use-unified-simulator \
  --build \
  --timeout-after 500 \
  --base-run-name master_0608 ${lwd}/simulator/scenario/jenkins/benchmarks_cn_aeb.lst
} 
rosparam set use_sim_time false
RunBenchmarkBagSimulation ${vehicle_name}
# rosparam set use_sim_time false
# RunBenchmarkBagSetCruiseSpeedSimulation ${vehicle_name}
rosparam set use_sim_time false
RunBenchmarkScenarioSetCruiseSpeedSimulation ${vehicle_name}
# rosparam set use_sim_time false
# RunBenchmarkScenarioSetCruiseSpeedManualSimulation ${vehicle_name}-manual