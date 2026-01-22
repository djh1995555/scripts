enum class ParkingHppStateMachine {
  // Hpp state machine status
  OFF,
  PASSIVE,
  STANDBY,
  READY,
  ACTIVE,
  RECOVERINTERRUPT,
  COMPLETE,
  TERMINATED,
  FAILURE,
  RECOVERINTERRUPT_TAKEOVER,
  PRESTANDBY,
  ROAMCALCULATE,
  ParkinOutStandby,
};
enum class ParkingHppActiveSubStatus {
  Default,
  Cruising,
  ParkingIn,
  ParkingOut,
};

enum class ApaRpaStateMachine {
  OFF,
  STANDBY,
  BACKGROUNDSEARCHING,
  SEARCHING,
  READY,
  ACTIVE,
  RECOVERINTERRUPT,
  COMPLETE,
  TERMINATED,
  FAILURE,
};
enum class ApaRpaActiveSubFunctionStatus {
  UNSPECIFIED,
  APAPARKIN,
  APAPARKOUT,
  RPAPARKIN,
  RPAPARKOUT,
  RPAMOVE,
};





enum class APAState {
    STATE_DEFAULT,
    STATE_NOT_READY,
    STATE_READY,
    STATE_STARTUP,
    STATE_ENTER_WIRE_CONTROL,
    STATE_RUNNING,
    STATE_STOPPING,
    STATE_GOAL_STOPPING,
    STATE_PULL_P_GEAR,
    STATE_COMPLETE,
    STATE_EXIT_WIRE_CONTROL,
    STATE_HANDSHAKE_ERROR,
    STATE_DRIVER_INTERFERENCE,
    STATE_ABNORMAL_STOPPING,
    STATE_ABNORMAL_PULL_P_GEAR,
    STATE_ABNORMAL_COMPLETE,
    STATE_OBSTACLE_STOPPING,
    STATE_OBSTACLE,
    STATE_RECOVER_INTERRUPT,
    STATE_TRAJECTORY_REQ_STOPPING,
    STATE_TRAJECTORY_REQ_COMPLETE,
    STATE_RECOVER_INTERRUPT_TAKEOVER,
    STATE_REPLAN_WITHOUT_BUTTON,
    STATE_RPA_MOVING_RUNNING,
    STATE_RPA_MOVING_OBSTACLE,
    STATE_WAIT_MAP,
    STATE_WAIT_PLANNER,
    STATE_USER_CONFIRM,
    STATE_WAIT_MIRROR_FOLD,
    STATE_PARKOUT_PRE_CHECK,
    STATE_WAIT_AUTO_REPLAN,
    STATE_PARKOUT_BACKWARD_PRE_CHECK,
    STATE_POPUP_TEACH
};
enum ControllerWorkingStatus {
  Unspecified = 0,
  Ready = 1,
  NotReady = 2,
  Handshake = 3,
  Running = 4,
  ParkingGoalComplete = 5,
  AbnormalComplete = 6,
  TriggerHmiNotification = 7,
  RecoveryInterruptStopping = 8,
  RecoveryInterruptTakeover = 9,
  CruiseGoalComplete = 10,
  ObstacleHolding = 11,
  GoalTransitonStatus = 12,
  ObstacleCheckStatus = 13,
  TrajectoryReqStopping = 15,
  ReplanWithoutButton = 16,
  WaitMap = 17,
  WaitPlanner = 18,
  WaitMirrorFold = 20,
  HandshakeError = 21,
  ExitWireControl = 22,
  ParkOutGoalComplete = 23,
  ParkOutGoalStopping = 24,
  WaitAutoReplan = 25,
  PopupTeach = 26,
  CollisionActionNarrowLotQuit = 31,
  WaitDynamicObstacleAway = 32,
  LeaveCarWaitDynamicObstacleAway = 33,
  UserDetectionHolding = 50,
};
enum WorkingStage {
    WS_STANDSTILL_STEERING,
    WS_DRIVE,
    WS_REVERSE,
    WS_DRIVE_TO_STOP,    // will stop in 3s
    WS_REVERSE_TO_STOP,  // will stop in 3s
    WS_STOP
};
enum SegmentStage {
    SEGMENT_DEFAULT,
    SEGMENT_INIT,
    SEGMENT_READY,
    SEGMENT_SWITCH_GEAR,
    SEGMENT_EPB_RELEASE,
    SEGMENT_STANDSTILL_STEERING,
    SEGMENT_STARTUP,
    SEGMENT_TRACKING,
    SEGMENT_STOPPING,
    SEGMENT_EMERGENCY_STOPPING,
    SEGMENT_FINISHED,
    SEGMENT_STUCKUP,
    SEGMENT_OBSTACLE_TARCKING,
    SEGMENT_OBSTACLE_STOPPING,
    SEGMENT_OBSTACLE_HOLDING,
    SEGMENT_ABNORMAL,
};








enum CollisionType {
  COLLISION_OD_TRAJECTORY_STOP,
  COLLISION_OD_PREDICT_STOP,
  COLLISION_SOURCE_PARKING_LOT_STOP,
  COLLISION_OBSTACLE_MAP_TRAJECTORY_COLLISION_STOP,
  COLLISION_OBSTACLE_MAP_TRAJECTORY_INVALID_STOP,
  COLLISION_OBSTACLE_MAP_PREDICT_STOP,
  COLLISION_WHEEL_STOP_OBSTACLE,
  COLLISION_LAST_SEG_STEREO_OBSTACLE_STOP,
  COLLISION_USER_REQ_STOP,
  COLLISION_RPA_PARK_SWITCH_TO_MOVING_STOP,
  COLLISION_TRAJ_PARALLEL_WHEEL_STOP,
  COLLISION_TRAJ_SLOT_OFFSET_TOO_LARGER,
  COLLISION_CONTROLLER_STOP,
  COLLISION_TRIGGER_POINT_SET_STOP,
  COLLISION_MAP_REQ_STOP,
  COLLISION_MAP_REQ_QUIT,
  COLLISION_NARROW_LOT_QUIT,
  COLLISION_MAP_SLOT_TOO_NARROW,
  COLLISION_PARK_OUT_WAIT_MIRROR_FOLD_STOP,
  COLLISION_PARKING_SPACE_END_LINE_STOP,
  COLLISION_PARKING_SPACE_ENTER_LINE_STOP,
  COLLISION_WHEEL_STOP_STOP,
  COLLISION_OBSTACLE_MAP_SPEED_LIMIT,
  COLLISION_PEDESTRIAN_OD_SPEED_LIMIT,
  COLLISION_OD_SPEED_LIMIT,
  COLLISION_POTENTIAL_OD_SPEED_LIMIT,
  COLLISION_POTENTIAL_PEDESTRIAN_OD_SPEED_LIMIT,
  COLLISION_SPEED_BUMP_SPEED_RISK,
  COLLISION_POTENTIAL,
  COLLISION_SPEED_BUMP_SPEED_FREE,
  COLLISION_TYPE_NONE,
};








std::map<uint32_t, std::string> quit_reason_map_ = {
      {0x00, "no request"},
      {0x01, "searching speed high "},
      {0x02, "searching time out"},
      {0x03, "driver change gear time out "},
      {0x04, "door open time out"},
      {0x05, "handshake fail "},
      {0x06, "driver require quit"},
      {0x07, "driver not release brake pedal time out"},
      {0x08, "driver seatbelt not tied time out"},
      {0x09, "press start button time out"},
      {0x0a, "reselect park in time out"},
      {0x0b, "driver interrupt brake pedal time out"},
      {0x0c, "driver interrupt gear"},
      {0x0d, "driver interrupt steering wheel"},
      {0x0e, "obstacle in trajectory time out "},
      {0x0f, "epb unlock"},
      {0x10, "parking total time out"},
      {0x11, "recover interrupt times overflow "},
      {0x12, "D/R times overflow"},
      {0x13, "control speed over limit"},
      {0x14, "path planning abnormal "},
      {0x15, "space limit"},
      {0x16, "slope out of limit "},
      {0x17, "adas function active"},
      {0x18, "bad weather "},
      {0x19, "crash "},
      {0x1a, "add system fail"},
      {0x1b, "sensor fail "},
      {0x1c, "related actuator fail"},
      {0x1d, "communication fail "},
      {0x1e, "sensor block"},
      {0x1f, "driver press app &rpa button time out"},
      {0x20, "driver actively paused time out"},
      {0x21, "driver leave phone supervision page time out"},
      {0x22, "phone distance exceed time out"},
      {0x23, "driver require ouit or kill the app"},
      {0x24, "phone bluetooth connect lost"},
      {0x25, "driver interrupt press gas pedal"},
      {0x26, "driver interrupt press brake pedal"},
      {0x27, "driver interrupt open door "},
      {0x28, "too close obstacles"},
      {0x29, "total distance over range"},
      {0x2a, "warning someone in car time out "},
      {0x2b, "none P status"},
      {0x2c, "rpa move not available"},
      {0x2d, "not press start button time out"},
      {0x2e, "choose park out direction time out "},
      {0x2f, "function off"},
      {0x30, "parking planning error"}};