#include "FSM/FSM.h"
#include <iostream>

FSM::FSM(CtrlComponents *ctrlComp)
    : _ctrlComp(ctrlComp)
{
    _stateList.invalid = nullptr;
    _stateList.passive = new State_Passive(_ctrlComp);
    _stateList.fixedStand = new State_FixedStand(_ctrlComp);
    _stateList.loco = new State_Loco(_ctrlComp);
    _stateList.amp = new State_AMP(_ctrlComp);
    _stateList.wbc = new State_WBC(_ctrlComp);
    initialize();
}

FSM::~FSM()
{
    _stateList.deletePtr();
}

void FSM::initialize()
{
    _currentState = _stateList.passive;
    _currentState->enter();
    _nextState = _currentState;
    _mode = FSMMode::NORMAL;

    std::cout << "Press **start** to enter position control mode..." << std::endl;
}

// FSM的运行函数，包含状态机的核心逻辑
void FSM::run()
{
    try
    {
        _startTime = getSystemTime(); // 获取当前系统时间，作为控制周期的起始时间

        _ctrlComp->sendRecv(); // 发送接收数据，更新传感器信息和机器人状态

        if (_mode == FSMMode::NORMAL) // 正常工作模式
        {
            _currentState->run();                            // 执行当前状态的控制逻辑
            _nextStateName = _currentState->checkChange();   // 检查是否需要切换状态，获取下一个状态的名称
            if (_nextStateName != _currentState->_stateName) // 如果下一个状态与当前状态不同，准备切换状态
            {
                _mode = FSMMode::CHANGE;
                _nextState = getNextState(_nextStateName);
                std::cout << "Switched from " << _currentState->_stateNameString
                          << " to " << _nextState->_stateNameString << std::endl;
            }
        }
        else if (_mode == FSMMode::CHANGE) // 切换模式
        {
            _currentState->exit();      // 清理内存，准备进入下一个状态
            _currentState = _nextState; // 切换到下一个状态
            _currentState->enter();     // 进入下一个状态，执行初始化逻辑
            _mode = FSMMode::NORMAL;
            _currentState->run(); // 进入下一个状态后，立即执行一次控制逻辑，确保状态切换的响应性
        }

        absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000)); // 等待控制周期结束，确保每个控制周期的时间一致，确保控制频率为50Hz
    }
    catch (const std::exception &e) // 捕获运行过程中可能出现的异常，防止程序崩溃，并输出错误信息
    {
        std::cerr << std::endl
                  << "Caught exception: " << e.what() << std::endl;
        _ctrlComp->exitFlag = true;
    }
}

FSMState *FSM::getNextState(FSMStateName stateName)
{
    switch (stateName)
    {
    case FSMStateName::INVALID:
        return _stateList.invalid;
        break;
    case FSMStateName::PASSIVE:
        return _stateList.passive;
        break;
    case FSMStateName::FIXEDSTAND:
        return _stateList.fixedStand;
        break;
    case FSMStateName::LOCO:
        return _stateList.loco;
    case FSMStateName::WBC:
        return _stateList.wbc;
    case FSMStateName::AMP:
        return _stateList.amp;
    default:
        return _stateList.invalid;
        break;
    }
}