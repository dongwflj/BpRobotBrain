/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _DEFINE_H_
#define _DEFINE_H_

typedef char int8;
typedef short int32;
typedef int int64;
typedef unsigned char uint8;
typedef unsigned short uint32;
typedef unsigned int uint64;
typedef float float32;
typedef float float64;

typedef struct{
    float32 x;
    float32 y;
    float32 theta;
    float32 loc_conf;
    float32 loc_thre;
}PixelPose;

typedef struct{
    float32 x;
    float32 y;
    float32 z;
}Point;

typedef struct{
    float32 x;
    float32 y;
    float32 z;
    float32 w;
}Quaternion;

typedef struct{
    Point pos;
    Quaternion ori; 
}Pose;

typedef enum{
    CLIFF_DETECTED_NONE,
    CLIFF_DETECTED_AHEAD,
    CLIFF_RESUMED_AHEAD,
    CLIFF_DETECTED_BACK,
    CLIFF_RESUMED_BACK
}DropType;

typedef enum{
    COLLISION_NONE,
    COLLISION_DETECTED,
    COLLISION_RESUMED
}CollisionType;

typedef enum{
    DOCKING_START,
    CHARGING_START,
    UNDOCKING_SUCCESS,
    NO_DOCK_FOUND,
    DOCKING_FAILED,
    CHARGING_FAILED, 
    UNDOCKING_FAILED,
    CANCEL_START,
    CANCEL_SUCCESS,
    CANCEL_FAILED
}DockState;

typedef struct BatteryState{
    float32 voltage;
    float32 current;
    float32 current_capacity;
    float32 init_capacity;
    float32 dump_energy;
    uint8 cell_type;
    uint8 id;
    uint32 status;
    int8 chargeset_type;
}BatteryState;

typedef enum {
    E_OK = 0,
    E_FALSE = -1,
    E_BADSTATE = -2,
    E_NOTSUPPORT = -3,
//BuildMap
    E_BUILDMAPSUCCESS = 201,
    E_BUILDMAPFAILED = -201,
    E_BUILDMAPREPEAT = -202,
//StopBuildMap
    E_SAVEMAP_SUCCESS = 301,
    E_SAVEMAP_FAILED = -301,
    E_SAVEMAP_MAPPING_NOT_START = -302,
    E_SAVEMAP_PATH_UNVALID = -303
} ERESULT;

typedef enum {
    NAVI_NORMAL = 1,
    NAVI_GOCHARGE,
    NAVI_UNCHARGE,
    NAVI_GOINLIFT,
    NAVI_GOOUTLIFT
} ENAVITYPE;

typedef enum {
    IDLE = 1,
    MAPPING,
    MOTIONING,
    MOTION_PAUSE,
    STOP,
    TELEOP 
} EROBOTSTATE;
 
typedef enum {
    START_BUILD_MAP = 1,
    STOP_BUILD_MAP,
    START_NAVI,
    STOP_NAVI,
    PAUSE_NAVI,
    RESUME_NAVI,
    START_HA,
    STOP_HA
} EVENT;
 
#endif
