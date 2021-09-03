/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _DEFINE_H_
#define _DEFINE_H_

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
 
#endif
