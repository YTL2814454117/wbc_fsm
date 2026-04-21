
#include <iostream>
#include <fstream>
#include "FSM/State_FixedStand.h"

State_FixedStand::State_FixedStand(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::FIXEDSTAND, "fixed stand") {}

// 进入状态时，记录当前关节位置作为起始位置，并设置目标位置为固定站立姿势
void State_FixedStand::enter()
{
    for (int i = 0; i < NUM_DOF; i++)
    {
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
        _startPos[i] = _lowState->motorState[i].q;
    }
    _phase = 0;
    _duration = 2.0;
    _fixedstand_complete_flag = false;
    std::string config_path = std::string(PROJECT_ROOT_DIR) + "/config/fixedpose.json";
    std::ifstream config_file(config_path);
    if (!config_file.is_open())
    {
        std::cerr << "[ERROR] Failed to open config file: " << config_path << std::endl;
        throw std::runtime_error("Cannot open config file");
    }
    try
    {
        json config = json::parse(config_file);
        _duration = config["duration"].get<float>();
    }
    catch (const std::exception &e)
    {
        std::cerr << "[ERROR] Failed to parse config file: " << e.what() << std::endl;
        throw;
    }
    config_file.close();
    std::cout << "Please make the robot stand first, stabilize it, then press **R2+A** to enter Locomode" << std::endl;
}

// 在状态运行时，根据预设的固定站立姿势目标位置和当前关节位置，进行线性差值控制，在2秒内逐渐将机器人移动到固定站立姿势
void State_FixedStand::run()
{

    // 线性差值控制 根据当前阶段（phase）计算每个关节的目标位置，并设置相应的控制参数
    _phase += _ctrlComp->dt / _duration;
    if (_phase >= 1)
    {
        _fixedstand_complete_flag = true;
    }
    _phase = _fixedstand_complete_flag ? 1 : _phase;

    for (int j = 0; j < NUM_DOF; j++)
    {
        _lowCmd->motorCmd[j].tau = 0;
        _lowCmd->motorCmd[j].q = (1 - _phase) * _startPos[j] + _phase * _targetPos[j]; // 线性差值计算目标位置
        _lowCmd->motorCmd[j].Kp = Kps[j];                                              // 设置电机刚度
        _lowCmd->motorCmd[j].Kd = Kds[j];                                              // 设置电机阻尼
    }
}

void State_FixedStand::exit()
{
    _phase = 0;
    _fixedstand_complete_flag = false;
}

FSMStateName State_FixedStand::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_B)
    {
        return FSMStateName::PASSIVE;
    }
    else if (_lowState->userCmd == UserCommand::R2_A)
    {
        return FSMStateName::AMP;
    }
    else if (_lowState->userCmd == UserCommand::SELECT)
    {
        throw std::runtime_error("exit..");
        return FSMStateName::PASSIVE;
    }
    else
    {
        return FSMStateName::FIXEDSTAND;
    }
}