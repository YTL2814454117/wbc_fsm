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
    // _stateList.wbc = new State_WBC(_ctrlComp);
    _stateList.wbc = new State_WBC_New(_ctrlComp);
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

    std::cout << "Press keyboard [1] to enter position control mode..." << std::endl;
}

// FSM的运行函数，包含状态机的核心逻辑
void FSM::run()
{
    try
    {
        _startTime = getSystemTime(); // 获取当前系统时间，作为控制周期的起始时间

        _ctrlComp->sendRecv(); // 发送接收数据，更新传感器信息和机器人状态
        handleDanceSelectionCommand();

        if (_mode == FSMMode::NORMAL) // 正常工作模式
        {
            _currentState->run();                            // 执行当前状态的控制逻辑
            _nextStateName = _currentState->checkChange();   // 检查是否需要切换状态，获取下一个状态的名称
            if (_nextStateName != _currentState->_stateName) // 如果下一个状态与当前状态不同，准备切换状态
            {
                _mode = FSMMode::CHANGE;
                _nextState = getNextState(_nextStateName);
                if (_currentState == _stateList.wbc && _nextStateName == FSMStateName::AMP)
                {
                    _stateList.amp->requestAutoLocoAfterEnter(3.0);
                    std::cout << "[FSM] WBC completed AMP handoff. AMP will stabilize for 3.0s before Loco." << std::endl;
                }
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

bool FSM::handleDanceSelectionCommand()
{
    if (!_ctrlComp->danceManager)
        return false;

    switch (_ctrlComp->lowState->userCmd)
    {
    case UserCommand::PREV_DANCE:
        _ctrlComp->danceManager->selectPrev();
        if (_currentState == _stateList.wbc)
            std::cout << "[DancePolicy] WBC is running. New selection will apply on the next WBC entry." << std::endl;
        _ctrlComp->lowState->userCmd = UserCommand::NONE;
        return true;
    case UserCommand::NEXT_DANCE:
        _ctrlComp->danceManager->selectNext();
        if (_currentState == _stateList.wbc)
            std::cout << "[DancePolicy] WBC is running. New selection will apply on the next WBC entry." << std::endl;
        _ctrlComp->lowState->userCmd = UserCommand::NONE;
        return true;
    case UserCommand::PRINT_DANCE:
        _ctrlComp->danceManager->printCurrentProfile();
        _ctrlComp->lowState->userCmd = UserCommand::NONE;
        return true;
    default:
        return false;
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
