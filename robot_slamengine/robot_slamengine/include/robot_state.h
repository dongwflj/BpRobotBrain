/*
* Copyright (c) 2021, Cloudminds, Inc.
* All rights reserved.
*
* author: Ewen Dong
*/

#ifndef _ROBOT_STATE_H_
#define _ROBOT_STATE_H_

#include <string>
#include "define.h"

namespace slamengine
{
class RobotFsm;
class RobotState {
public:
    virtual ~RobotState() {}
    void SetContext(RobotFsm *context) {
        this->context_ = context;
    }

    virtual ERESULT StartBuildMap(const std::string& task_id);
    virtual ERESULT StopBuildMap(const std::string& task_id);
    virtual ERESULT PauseBuildMap(const std::string& task_id);
    virtual ERESULT ResumeBuildMap(const std::string& task_id);
    // Map
    ERESULT SaveMap(const std::string& task_id, const std::string& map_name);
    ERESULT LoadMap(const std::string& task_id, const std::string& map_name);
    // Navi
    virtual ERESULT StartNavi(const std::string& task_id, ENAVITYPE type, const std::string& goal_name, const PixelPose& goal_pose);
    virtual ERESULT StopNavi(const std::string& task_id);
    virtual ERESULT PauseNavi(const std::string& task_id);
    virtual ERESULT ResumeNavi(const std::string& task_id);
    virtual ERESULT NotifyDropEvent(DropType type){}
    virtual ERESULT NotifyCollisionEvent(CollisionType type){}
    virtual ERESULT NotifyEStopEvent(EmergencyType type){}
    virtual ERESULT NotifyChargingEvent(const BatteryState& state){}
    virtual ERESULT NotifyCriticalHwErrEvent(){}
    virtual ERESULT InitDoneEvent();
    virtual ERESULT NaviDoneEvent(const std::string& task_id, int32 state, const std::string& description);
    virtual ERESULT NaviActiveEvent(const std::string& task_id, bool active);
    virtual ERESULT NaviProgressEvent(const std::string& task_id, const Pose& curr_pose);
    virtual ERESULT Resume();

    class BlackBoard {
    public:
        // If we needn't return prev state, we need keep from_state_ is nullptr
        BlackBoard() { from_state_ = nullptr; }
        std::string task_id_;
        ENAVITYPE type_;
        std::string goal_name_;
        PixelPose goal_pose_;
        RobotState* from_state_;
        std::map<std::string, std::string> params_;
    };
    ERESULT SetBlackBoard(const BlackBoard& board) { black_board_ = board; }
    const BlackBoard& GetBlackBoard() { return black_board_; }

protected:
    RobotFsm* context_;
    BlackBoard black_board_;
};

} // end_ns

#endif
