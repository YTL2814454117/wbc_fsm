#include <iostream>
#include "FSM/State_WBC.h"
#include "common/read_traj.h"
#include <fstream>
#include <algorithm>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// 基于模仿学习的wbc实现
State_WBC::State_WBC(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::WBC, "wbc")
{

    std::string config_path = std::string(PROJECT_ROOT_DIR) + "/config/wbc.json";
    std::ifstream config_file(config_path);
    if (!config_file.is_open())
    {
        std::cerr << "[ERROR] Failed to open config file: " << config_path << std::endl;
        throw std::runtime_error("Cannot open config file");
    }

    try
    {
        json config = json::parse(config_file);
        std::string base_path = std::string(PROJECT_ROOT_DIR) + "/";
        _model_path = base_path + config["model_path"].get<std::string>();
        _folder_path = base_path + config["motion_path"].get<std::string>();
        _anchor_terminate_thresh = config["safe_projgravity_threshold"].get<float>();
        _start_refer_idx = config["start_idx"].get<int>();
        _pause_refer_idx = config["pause_idx"].get<int>();
        _end_refer_idx = config["end_idx"].get<int>();
        std::cout << "[Config] Model path: " << _model_path << std::endl;
        std::cout << "[Config] Folder path: " << _folder_path << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[ERROR] Failed to parse config file: " << e.what() << std::endl;
        throw;
    }
    config_file.close();

    // 加载二进制运动数据
    _bin_data_loaded = BinaryArrayReader::readBinFilesFromFolder(
        _folder_path,
        _body_ang_vel_w, _body_ang_vel_w_shape,
        _body_lin_vel_w, _body_lin_vel_w_shape,
        _body_pos_w, _body_pos_w_shape,
        _body_quat_w, _body_quat_w_shape,
        _fps, _fps_shape,
        _joint_pos, _joint_pos_shape,
        _joint_vel, _joint_vel_shape);
    _motion_frame_count = _joint_pos_shape[0];
    if (_bin_data_loaded)
    {
        std::cout << "[SUCCESS] Loaded motion data from: " << _folder_path << std::endl;
        std::cout << "Total motion frames: " << _motion_frame_count << std::endl;
    }
    else
    {
        std::cerr << "[ERROR] Failed to load some binary data!" << std::endl;
    }
    _loadPolicy();
}

void State_WBC::_init_buffers()
{
    _last_refer_idx = 0;
    for (int i = 0; i < this->_actor_state_history_length; ++i)
    {
        _observations_compute();
    }
}

void State_WBC::_loadPolicy()
{
    auto available_providers = Ort::GetAvailableProviders();
    _session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);     // 开启最高级别的图优化
    _session = std::make_unique<Ort::Session>(_env, _model_path.c_str(), _session_options); // 加载ONNX模型

    Ort::TypeInfo input_type = _session->GetInputTypeInfo(0);
    auto input_shapes = input_type.GetTensorTypeAndShapeInfo().GetShape(); // 获取输入张量的形状信息
    Ort::TypeInfo output_type = _session->GetOutputTypeInfo(0);
    auto output_shapes = output_type.GetTensorTypeAndShapeInfo().GetShape(); // 获取输出张量的形状信息

    // ==========================================
    // 🔍 调试信息打印块 开始
    // ==========================================
    std::cout << "\n[DEBUG] =======================================\n";
    std::cout << "[DEBUG] 成功加载 ONNX 模型路径: " << _model_path << "\n";

    // 打印输入维度
    std::cout << "[DEBUG] 输入张量 (Observation) Shape: [";
    for (size_t i = 0; i < input_shapes.size(); ++i)
    {
        std::cout << input_shapes[i] << (i == input_shapes.size() - 1 ? "" : ", ");
    }
    std::cout << "]\n";

    // 打印输出维度
    std::cout << "[DEBUG] 输出张量 (Action) Shape: [";
    for (size_t i = 0; i < output_shapes.size(); ++i)
    {
        std::cout << output_shapes[i] << (i == output_shapes.size() - 1 ? "" : ", ");
    }
    std::cout << "]\n";
    std::cout << "[DEBUG] =======================================\n\n";
    // ==========================================
    // 🔍 调试信息打印块 结束
    // ==========================================

    _obs_size_ = input_shapes[1];
    _action_size_ = output_shapes[1];
    _action = std::vector<float>(_action_size_, 0.0f);
}

void State_WBC::_observations_compute()
{
    std::vector<float> base_quat = std::vector<float>(4, 0.0f); // 读取IMU四元数
    base_quat = {
        _lowState->imu.quaternion[0],  // w
        _lowState->imu.quaternion[1],  // x
        _lowState->imu.quaternion[2],  // y
        _lowState->imu.quaternion[3]}; // z
    std::vector<float> projected_gravity(3);
    projected_gravity = QuatRotateInverse(base_quat, this->_gravity_vec); // 根据机器人姿态四元数 q，把世界坐标中的重力向量 v 变换到机器人身体坐标系。
    auto obs_projected_gravity = projected_gravity;

    // 读取机器人腰部 3 个关节的当前位置（角度），存入 waist_yrp 向量。
    std::vector<float> waist_yrp(3);
    for (int i = 0; i < 3; i++)
    {
        waist_yrp[i] = _lowState->motorState[_waist_yrp_idx[i]].q;
    }
    Eigen::Matrix3f R_b2w = rotz(waist_yrp[0]) * rotx(waist_yrp[1]) * roty(waist_yrp[2]); // 腰相对底座的旋转矩阵，base to waist
    Eigen::Matrix3f R_base = matrix_from_quat(base_quat);                                 // 把 IMU 给出的 base 四元数转换成旋转矩阵。
    Eigen::Matrix3f R_waist = R_base * R_b2w;                                             // 先有身体base 朝向，再叠加腰关节旋转。
    std::vector<float> waist_quat = quat_from_matrix(R_waist);                            // 把腰部旋转矩阵转换回四元数，得到腰部在世界坐标系下的姿态表示。

    std::vector<float> dof_pos_vec;
    dof_pos_vec.reserve(NUM_DOF); // 提前申请 NUM_DOF 个元素空间。
    for (int i = 0; i < NUM_DOF; ++i)
    {
        dof_pos_vec.push_back(_lowState->motorState[dof_mapping[i]].q - this->_default_dof_pos[dof_mapping[i]]); // 获取所有关节当前位置，并转换成相对默认站姿的偏移量
    }

    std::vector<float> dof_vel_vec;
    dof_vel_vec.reserve(NUM_DOF);
    for (int i = 0; i < NUM_DOF; ++i)
    {
        dof_vel_vec.push_back(_lowState->motorState[dof_mapping[i]].dq); // 获取所有关节当前速度，并转换成相对默认站姿的偏移量
    }

    auto body_ang_vel = std::vector<float>({static_cast<float>(_lowState->imu.gyroscope[0]),
                                            static_cast<float>(_lowState->imu.gyroscope[1]),
                                            static_cast<float>(_lowState->imu.gyroscope[2])}); // 获取身体的三轴角速度
    body_ang_vel[0] = body_ang_vel[0] * scale_lin_vel;
    body_ang_vel[1] = body_ang_vel[1] * scale_lin_vel;
    body_ang_vel[2] = body_ang_vel[2] * scale_ang_vel; // 对角速度进行缩放，缩放因子似乎都是1.0f，暂时没有实际效果，可能需要修改

    for (int i = 0; i < NUM_DOF; i++)
    {
        dof_pos_vec[i] = dof_pos_vec[i] * scale_dof_pos;
        dof_vel_vec[i] = dof_vel_vec[i] * scale_dof_vel; // 进行缩放，缩放因子似乎都是1.0f，暂时没有实际效果，可能需要修改
    }

    std::vector<float> current_robot_state;                                                        // 构建当前机器人状态向量，包含身体角速度、投影重力、关节位置和速度等信息，作为RL模型的输入
    current_robot_state.reserve(3 + 3 + dof_pos_vec.size() + dof_vel_vec.size() + _action.size()); // 预先分配足够的空间，避免多次动态扩容带来的性能损失
    current_robot_state.insert(current_robot_state.end(), body_ang_vel.begin(), body_ang_vel.end());
    current_robot_state.insert(current_robot_state.end(), obs_projected_gravity.begin(), obs_projected_gravity.end());
    current_robot_state.insert(current_robot_state.end(), dof_pos_vec.begin(), dof_pos_vec.end());
    current_robot_state.insert(current_robot_state.end(), dof_vel_vec.begin(), dof_vel_vec.end());
    current_robot_state.insert(current_robot_state.end(), _action.begin(), _action.end()); // 把上一步计算得到的动作也加入到当前状态向量中，可能是为了让模型能够根据上一步的动作来调整当前的动作输出，实现更平滑的控制效果

    // 维护一个固定长度的历史状态缓冲区_robot_state_obs_buf，存储最近几帧的机器人状态信息，以提供给RL模型作为输入，帮助模型捕捉时间动态特征
    _robot_state_obs_buf.erase(_robot_state_obs_buf.begin(),
                               _robot_state_obs_buf.begin() + _robot_state_dim);
    _robot_state_obs_buf.insert(_robot_state_obs_buf.end(),
                                current_robot_state.begin(),
                                current_robot_state.end());

    // 定义了4个Lambda匿名函数，从拍扁的一维连续内存数组中，通过索引计算，提取出特定帧（Frame）、特定刚体（Link）或关节（DoF）的三维/多维张量数据。
    auto get_pos = [this](int frame_idx, int link_idx) -> std::vector<float> // 获取特定帧和刚体的位置信息，返回一个包含x、y、z坐标的三维向量
    {
        int num_links = _body_pos_w_shape[1];
        int base = frame_idx * num_links * 3 + link_idx * 3;
        return {_body_pos_w[base], _body_pos_w[base + 1], _body_pos_w[base + 2]};
    };
    auto get_quat = [this](int frame_idx, int link_idx) -> std::vector<float> // 获取指定帧、指定刚体的 3D 空间姿态
    {
        int num_links = _body_quat_w_shape[1];
        int base = frame_idx * num_links * 4 + link_idx * 4;
        return {_body_quat_w[base], _body_quat_w[base + 1], _body_quat_w[base + 2], _body_quat_w[base + 3]};
    };
    auto get_dof_pos = [this](int frame_idx) -> std::vector<float> // 获取指定帧的所有关节位置数据，返回一个包含所有关节位置的向量
    {
        int num_dofs = _joint_pos_shape[1];
        int base = frame_idx * num_dofs;
        std::vector<float> dof_pos(num_dofs);
        for (int i = 0; i < num_dofs; ++i)
        {
            dof_pos[i] = _joint_pos[base + i];
        }
        return dof_pos;
    };
    auto get_dof_vel = [this](int frame_idx) -> std::vector<float> // 获取指定帧的所有关节当前角速度
    {
        int num_dofs = _joint_vel_shape[1];
        int base = frame_idx * num_dofs;
        std::vector<float> dof_vel(num_dofs);
        for (int i = 0; i < num_dofs; ++i)
        {
            dof_vel[i] = _joint_vel[base + i];
        }
        return dof_vel;
    };

    std::vector<float> cur_refer_anchor_pos;
    std::vector<float> cur_refer_dof_pos;
    std::vector<float> cur_refer_dof_vel;
    std::vector<float> cur_refer_anchor_quat;
    std::vector<float> ref_yaw_quat;
    std::vector<float> ref_yaw_quat_conj;
    std::vector<float> yaw_quat_delta;
    std::vector<float> aligned_cur_refer_anchor_quat;
    std::vector<float> motion_projected_gravity;
    std::vector<float> base_yaw_quat = yaw_quat(base_quat);
    std::vector<float> last_refer_anchor_pos;
    std::vector<float> last_refer_anchor_quat;

    std::vector<float> tgt_anchor_ori_b_flat;
    std::vector<float> tgt_anchor_pos_b_flat;
    std::vector<float> tgt_dof_pos_flat;
    std::vector<float> tgt_dof_vel_flat;

    // 目前该循环只执行一次，因为_mimic_obs_predictive_horizon被设置为1了
    for (int i = 0; i < _mimic_obs_predictive_horizon; i++)
    {

        // 计算出未来第i步对应的动作捕捉数据帧索引 idx，以及上一个动作捕捉数据帧索引 last_idx。根据当前是否处于暂停状态，来决定是否使用相同的帧索引，或者正常递增帧索引以获取未来的参考数据。
        int idx = _refer_idx + i * _frame_interval;
        int last_idx = idx - _frame_interval;
        if (_pause_flag)
        {
            idx = _refer_idx;
            last_idx = _refer_idx;
        }
        if (idx >= _end_refer_idx)
            idx = _end_refer_idx;
        else if (idx <= 1)
            idx = 1;
        if (last_idx >= _end_refer_idx)
            last_idx = _end_refer_idx;
        else if (last_idx <= 1)
            last_idx = 1;

        cur_refer_anchor_pos = get_pos(idx, _anchor_idx);
        cur_refer_dof_pos = get_dof_pos(idx);
        if (_pause_flag)
        {
            cur_refer_dof_vel = std::vector<float>(NUM_DOF, 0.0f);
        }
        else
        {
            cur_refer_dof_vel = get_dof_vel(idx);
        }

        // 过滤掉绝对偏航角（Yaw），把参考轨迹旋转到和机器人当前的朝向一致。
        cur_refer_anchor_quat = get_quat(idx, _anchor_idx);
        ref_yaw_quat = yaw_quat(cur_refer_anchor_quat);
        ref_yaw_quat_conj = quat_conjugate(ref_yaw_quat);
        yaw_quat_delta = quat_multiply(base_yaw_quat, ref_yaw_quat_conj);
        aligned_cur_refer_anchor_quat = quat_multiply(yaw_quat_delta, cur_refer_anchor_quat);

        last_refer_anchor_pos = get_pos(last_idx, _anchor_idx);
        last_refer_anchor_quat = get_quat(last_idx, _anchor_idx);
        if (i == 0)
            motion_projected_gravity = quat_apply_inverse(aligned_cur_refer_anchor_quat, _gravity_vec);

        // 计算的是相邻两帧参考动作之间的局部相对位姿变化
        auto [cur_target_pos, cur_target_quat] = subtract_frame_transforms(
            last_refer_anchor_pos,
            last_refer_anchor_quat,
            &cur_refer_anchor_pos,
            &cur_refer_anchor_quat);

        // 6D 连续旋转表示
        Eigen::Matrix3f mat = matrix_from_quat(cur_target_quat);
        std::vector<float> tgt_anchor_ori_b(6);
        tgt_anchor_ori_b[0] = mat(0, 0);
        tgt_anchor_ori_b[1] = mat(0, 1);
        tgt_anchor_ori_b[2] = mat(1, 0);
        tgt_anchor_ori_b[3] = mat(1, 1);
        tgt_anchor_ori_b[4] = mat(2, 0);
        tgt_anchor_ori_b[5] = mat(2, 1);

        // 将当前帧的参考关节位置、关节速度、锚点位置和姿态等信息，按照顺序依次存入对应的扁平化向量中，为后续构建RL模型输入做准备。
        tgt_dof_pos_flat.insert(tgt_dof_pos_flat.end(), cur_refer_dof_pos.begin(), cur_refer_dof_pos.end());
        tgt_dof_vel_flat.insert(tgt_dof_vel_flat.end(), cur_refer_dof_vel.begin(), cur_refer_dof_vel.end());
        tgt_anchor_pos_b_flat.insert(tgt_anchor_pos_b_flat.end(), cur_target_pos.begin(), cur_target_pos.end());
        tgt_anchor_ori_b_flat.insert(tgt_anchor_ori_b_flat.end(), tgt_anchor_ori_b.begin(), tgt_anchor_ori_b.end());
    }

    // 打包模仿学习的观察向量，包含下一帧的参考关节位置、关节速度、锚点位置和姿态等信息
    std::vector<float> mimic_obs;
    mimic_obs.insert(mimic_obs.end(), tgt_dof_pos_flat.begin(), tgt_dof_pos_flat.end());
    mimic_obs.insert(mimic_obs.end(), tgt_dof_vel_flat.begin(), tgt_dof_vel_flat.end());
    mimic_obs.insert(mimic_obs.end(), tgt_anchor_pos_b_flat.begin(), tgt_anchor_pos_b_flat.end());
    mimic_obs.insert(mimic_obs.end(), tgt_anchor_ori_b_flat.begin(), tgt_anchor_ori_b_flat.end());

    float anchor_proj_gravity_error = std::abs(motion_projected_gravity[2] - projected_gravity[2]);
    // std::cout << "anchor_proj_gravity_error: " << anchor_proj_gravity_error << std::endl;
    if (anchor_proj_gravity_error > _anchor_terminate_thresh)
    {
        _terminate_flag = true;
        std::cout << "current _anchor_terminate_thresh: " << _anchor_terminate_thresh << std::endl;
        std::cout << "[Warning] Large anchor projected gravity error: " << anchor_proj_gravity_error << std::endl;
    }

    this->_observation.clear();
    this->_observation = std::vector<float>();
    this->_observation.reserve(
        _robot_state_obs_buf.size() + mimic_obs.size());
    this->_observation.insert(this->_observation.end(), mimic_obs.begin(), mimic_obs.end());
    this->_observation.insert(this->_observation.end(), _robot_state_obs_buf.begin(), _robot_state_obs_buf.end());

    for (int i = 0; i < this->_observation.size(); i++)
    {
        this->_observation[i] = std::max(-clip_observations, std::min(this->_observation[i], clip_observations));
    }
}

void State_WBC::_action_compute()
{
    try
    {
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeCPU);

        std::vector<Ort::Value> input_tensors;
        std::vector<int64_t> obs_shape = {1,
                                          _robot_state_dim * _actor_state_history_length + _reference_dim * _mimic_obs_predictive_horizon};

        input_tensors.push_back(Ort::Value::CreateTensor<float>(
            memory_info,
            _observation.data(),
            _observation.size(),
            obs_shape.data(),
            obs_shape.size()));

        auto output_tensors = _session->Run(
            Ort::RunOptions{nullptr},
            _input_names.data(),
            input_tensors.data(),
            input_tensors.size(),
            _output_names.data(),
            1);

        float *actions = output_tensors[0].GetTensorMutableData<float>();
        std::memcpy(_action.data(), actions, _action.size() * sizeof(float));

        std::vector<float> actions_scaled(_action.size());
        for (int i = 0; i < _action.size(); i++)
        {
            _action[i] = std::max(-clip_actions, std::min(_action[i], clip_actions));
            actions_scaled[i] = _action[i] * action_scale + _default_dof_pos[dof_mapping[i]]; // action_scale
            this->_joint_q[dof_mapping[i]] = actions_scaled[i];
        }
    }
    catch (const Ort::Exception &e)
    {
        std::cerr << "ONNX Runtime error: " << e.what() << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Standard exception: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "Unknown error occurred" << std::endl;
    }
}

void State_WBC::enter()
{
    _pause_flag = false;
    _terminate_flag = false;
    _pause_curr_flag = false; // 清除历史标记
    _refer_idx = _start_refer_idx;
    _last_refer_idx = _refer_idx; // 设置动作帧数的索引
    if (_pause_refer_idx < 0)
    {
        _pause_refer_idx = 0;
    }
    if (_end_refer_idx < 0 || _end_refer_idx < _start_refer_idx)
    {
        if (_end_refer_idx < _start_refer_idx)
        {
            std::cout << "[WARNING]: end_idx is smaller than start_idx, defaulting to the length of the motion." << std::endl;
        }
        _end_refer_idx = _motion_frame_count - 1;
    }
    // 对当前状态进行软锁定
    for (int i = 0; i < NUM_DOF; i++)
    {
        _lowCmd->motorCmd[i].mode = 10;                      // 进入 PD 位置控制模式
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q; // 将当前关节位置作为初始目标位置
        _lowCmd->motorCmd[i].dq = 0;                         // 速度目标设为0
        _lowCmd->motorCmd[i].tau = 0;                        // 力矩目标设为0
        _lowCmd->motorCmd[i].Kp = this->dof_Kps[i];
        _lowCmd->motorCmd[i].Kd = this->dof_Kds[i];
        this->_targetPos_rl[i] = this->_default_dof_pos[i];
        this->_last_targetPos_rl[i] = _lowState->motorState[i].q;
        this->_joint_q[i] = this->_default_dof_pos[i];
    }
    // _init_buffers();
}

void State_WBC::run()
{
    if (!_pause_flag)
    {
        _refer_idx++;
    }
    else
    {
        if (!_pause_curr_flag)
            _refer_idx = _pause_refer_idx;
    }
    if (_refer_idx >= _end_refer_idx)
    {
        _refer_idx = _end_refer_idx;
    }
    _observations_compute();
    _action_compute();
    memcpy(this->_targetPos_rl, this->_joint_q, sizeof(this->_joint_q));

    for (int j = 0; j < NUM_DOF; j++)
    {
        _lowCmd->motorCmd[j].mode = 10;
        _lowCmd->motorCmd[j].q = _targetPos_rl[j];
        _lowCmd->motorCmd[j].dq = 0;
        _lowCmd->motorCmd[j].tau = 0;
        _lowCmd->motorCmd[j].Kp = this->dof_Kps[j];
        _lowCmd->motorCmd[j].Kd = this->dof_Kds[j];
        this->_last_targetPos_rl[j] = _targetPos_rl[j];
    }
    _last_refer_idx = _refer_idx;
    std::string pause_string = _pause_flag ? " | Press R1 to resume..." : " | Press R2 to pause...";
    std::cout << "\r[State_WBC] Running WBC state. Refer idx: " << _refer_idx << "/" << _end_refer_idx << pause_string << std::flush;
}

void State_WBC::exit()
{
    std::cout << "[State_WBC] Exiting WBC state." << std::endl;
}

FSMStateName State_WBC::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_B)
    {
        return FSMStateName::PASSIVE;
    }
    else if (_terminate_flag)
    {
        return FSMStateName::PASSIVE;
    }
    else if (_lowState->userCmd == UserCommand::R2_A)
    {
        return FSMStateName::AMP;
    }
    else if (_lowState->userCmd == UserCommand::R2 && !_pause_flag)
    {
        _pause_flag = true;
        std::cout << std::endl
                  << "WBC Pause" << std::endl;
        return FSMStateName::WBC;
    }
    else if (_lowState->userCmd == UserCommand::L2 && !_pause_flag)
    {
        _pause_flag = true;
        _pause_curr_flag = true;
        std::cout << std::endl
                  << "WBC Pause" << std::endl;
        return FSMStateName::WBC;
    }
    else if (_lowState->userCmd == UserCommand::R1 && _pause_flag)
    {
        _pause_flag = false;
        _pause_curr_flag = false;
        std::cout << std::endl
                  << "WBC Resume" << std::endl;
        return FSMStateName::WBC;
    }
    else if (_lowState->userCmd == UserCommand::SELECT)
    {
        throw std::runtime_error("exit..");
        return FSMStateName::PASSIVE;
    }
    else
    {
        return FSMStateName::WBC;
    }
}
