// APA&RPA Status
const int kRpaOff = 0;
const int kRpaStandby = 1;
const int kRpaBackSearching = 2;
const int kRpaSearching = 3;
const int kRpaReady = 4;
const int kRpaActive = 5;
const int kRpaRecovery = 6;
const int kRpaComplete = 7;
const int kRpaTerminated = 8;
const int kRpaFailure = 9;

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