
#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <string>
#include <iostream>
#include <unistd.h>
#include "control/CtrlComponents.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "common/enumClass.h"
#include "common/mathTools.h"
#include "common/mathTypes.h"
#include "common/timeMarker.h"
#include "interface/CmdPanel.h"

// FSM状态基类，定义了状态的基本接口和成员变量
class FSMState
{
public:
    FSMState(CtrlComponents *ctrlComp, FSMStateName stateName, std::string stateNameString);
    virtual ~FSMState() = default;

    virtual void enter() = 0;
    virtual void run() = 0;
    virtual void exit() = 0;                                             // 这三个纯虚函数要求每个具体状态类必须实现它们，分别对应状态的进入、运行和退出逻辑
    virtual FSMStateName checkChange() { return FSMStateName::INVALID; } // 默认实现，返回INVALID表示不切换状态，具体状态类可以重写这个函数来实现状态切换的条件判断

    // 定义公共的状态名称和状态字符串，方便调试和状态切换的识别
    FSMStateName _stateName;
    std::string _stateNameString;
    LowlevelCmd *_lowCmd;
    LowlevelState *_lowState;
    UserValue _userValue;

protected:
    CtrlComponents *_ctrlComp;
    FSMStateName _nextStateName;
};

#endif // FSMSTATE_H