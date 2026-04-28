#include <iostream>
#include "FSM/State_WBC_New.h"
#include "common/read_traj.h"
#include <fstream>
#include <algorithm>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

State_WBC_New::State_WBC_New(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::WBC, "wbc_new")
{
    // 加载配置文件
    std::string config_path = std::string(PROJECT_ROOT_DIR) + "/config/wbc_new.json";
    std::ifstream config_file(config_path);
    if (!config_file.is_open())
        throw std::runtime_error("Cannot open config file");

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
    }
    catch (const std::exception &e)
    {
        throw;
    }
    config_file.close();

    // 初始化 PD 参数 (根据 config 中的 stiffness 和 damping 映射到电机 ID)
    double config_stiffness[NUM_DOF] = {
        40.2, 99.1, 40.2, 99.1, 28.5, 28.5, 40.2, 99.1, 40.2, 99.1, 28.5, 28.5,
        40.2, 28.5, 28.5, 14.3, 14.3, 14.3, 14.3, 14.3, 16.8, 16.8, 14.3, 14.3, 14.3, 14.3,
        14.3, 16.8, 16.8};
    double config_damping[NUM_DOF] = {
        2.56, 6.31, 2.56, 6.31, 1.81, 1.81, 2.56, 6.31, 2.56, 6.31, 1.81, 1.81,
        2.56, 1.81, 1.81, 0.907, 0.907, 0.907, 0.907, 0.907, 1.07, 1.07, 0.907, 0.907, 0.907, 0.907,
        0.907, 1.07, 1.07};

    for (int i = 0; i < NUM_DOF; ++i)
    {
        int motor_id = dof_mapping[i];
        dof_Kps[motor_id] = config_stiffness[i];
        dof_Kds[motor_id] = config_damping[i];
    }

    // 加载动作捕捉二进制文件
    std::vector<float> unused_lin_vel;
    std::vector<uint32_t> unused_lin_vel_shape;
    std::vector<int64_t> unused_fps;
    std::vector<uint32_t> unused_fps_shape;

    _bin_data_loaded = BinaryArrayReader::readBinFilesFromFolder(
        _folder_path,
        _body_ang_vel_w, _body_ang_vel_w_shape,
        unused_lin_vel, unused_lin_vel_shape,
        _body_pos_w, _body_pos_w_shape,
        _body_quat_w, _body_quat_w_shape,
        unused_fps, unused_fps_shape,
        _joint_pos, _joint_pos_shape,
        _joint_vel, _joint_vel_shape);

    if (!_bin_data_loaded || _joint_pos_shape.size() != 2 || _joint_vel_shape.size() != 2 ||
        _body_quat_w_shape.size() != 3 || _joint_pos_shape[1] != NUM_DOF || _joint_vel_shape[1] != NUM_DOF)
    {
        throw std::runtime_error("Invalid mimic motion bin data shape");
    }

    _motion_frame_count = _joint_pos_shape[0];
    _loadPolicy();
}

void State_WBC_New::_init_buffers()
{
    _last_refer_idx = 0;
    _observations_compute();
}

void State_WBC_New::_loadPolicy()
{
    _session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    _session = std::make_unique<Ort::Session>(_env, _model_path.c_str(), _session_options);
    auto input_shapes = _session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    auto output_shapes = _session->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    _obs_size_ = input_shapes.size() > 1 && input_shapes[1] > 0 ? input_shapes[1] : _obs_dim;
    _action_size_ = output_shapes.size() > 1 && output_shapes[1] > 0 ? output_shapes[1] : _policy_action_dim;
    if (_obs_size_ != _obs_dim || _action_size_ != _policy_action_dim)
    {
        throw std::runtime_error("ONNX policy shape mismatch for State_WBC_New");
    }
    _action = std::vector<float>(_action_size_, 0.0f);
}

std::vector<float> State_WBC_New::_current_torso_quat() const
{
    std::vector<float> base_quat = {
        static_cast<float>(_lowState->imu.quaternion[0]),
        static_cast<float>(_lowState->imu.quaternion[1]),
        static_cast<float>(_lowState->imu.quaternion[2]),
        static_cast<float>(_lowState->imu.quaternion[3])};

    Eigen::Matrix3f torso_rot =
        matrix_from_quat(base_quat) *
        rotz(static_cast<float>(_lowState->motorState[_waist_yaw_motor_id].q)) *
        rotx(static_cast<float>(_lowState->motorState[_waist_roll_motor_id].q)) *
        roty(static_cast<float>(_lowState->motorState[_waist_pitch_motor_id].q));

    return quat_from_matrix(torso_rot);
}

std::vector<float> State_WBC_New::_reference_root_quat(int frame_idx) const
{
    int idx = std::clamp(frame_idx, 0, _motion_frame_count - 1);
    int num_bodies = static_cast<int>(_body_quat_w_shape[1]);
    int base = (idx * num_bodies + _root_idx) * 4;
    return {_body_quat_w[base], _body_quat_w[base + 1], _body_quat_w[base + 2], _body_quat_w[base + 3]};
}

std::vector<float> State_WBC_New::_reference_torso_quat(int frame_idx) const
{
    int idx = std::clamp(frame_idx, 0, _motion_frame_count - 1);
    int base = idx * NUM_DOF;
    const float yaw = _joint_pos[base + _waist_yaw_policy_idx];
    const float roll = _joint_pos[base + _waist_roll_policy_idx];
    const float pitch = _joint_pos[base + _waist_pitch_policy_idx];

    Eigen::Matrix3f torso_rot =
        matrix_from_quat(_reference_root_quat(idx)) *
        rotz(yaw) *
        rotx(roll) *
        roty(pitch);

    return quat_from_matrix(torso_rot);
}

void State_WBC_New::_observations_compute()
{
    std::vector<float> robot_torso_quat = _current_torso_quat();

    // 观测项: 机器人角速度
    std::vector<float> body_ang_vel = {
        static_cast<float>(_lowState->imu.gyroscope[0]) * scale_lin_vel,
        static_cast<float>(_lowState->imu.gyroscope[1]) * scale_lin_vel,
        static_cast<float>(_lowState->imu.gyroscope[2]) * scale_ang_vel};

    // 观测项: 关节位置和速度偏差 (按网络顺序 i 对应映射)
    std::vector<float> dof_pos_rel(NUM_DOF);
    std::vector<float> dof_vel_rel(NUM_DOF);
    for (int i = 0; i < NUM_DOF; ++i)
    {
        int motor_id = dof_mapping[i];

        // ⚠️ 关键修正：减去的是 _default_dof_pos[i]，千万别写错！
        dof_pos_rel[i] = (_lowState->motorState[motor_id].q - _default_dof_pos[i]) * scale_dof_pos;
        dof_vel_rel[i] = _lowState->motorState[motor_id].dq * scale_dof_vel;
    }

    // 参考轨迹处理
    int idx = std::clamp((_pause_flag ? (int)_pause_refer_idx : (int)_refer_idx), 1, (int)_motion_frame_count - 1);

    auto get_dof_pos = [this](int f_idx)
    {
        std::vector<float> p(NUM_DOF);
        for (int i = 0; i < NUM_DOF; ++i)
            p[i] = _joint_pos[f_idx * NUM_DOF + i];
        return p;
    };
    auto get_dof_vel = [this](int f_idx)
    {
        std::vector<float> v(NUM_DOF);
        for (int i = 0; i < NUM_DOF; ++i)
            v[i] = _joint_vel[f_idx * NUM_DOF + i];
        return v;
    };

    std::vector<float> ref_dof_pos = get_dof_pos(idx);
    std::vector<float> ref_dof_vel = (_pause_flag) ? std::vector<float>(NUM_DOF, 0.0f) : get_dof_vel(idx);

    // 6D 姿态处理
    std::vector<float> ref_anchor_quat = _reference_torso_quat(idx);
    std::vector<float> aligned_ref_anchor_quat = quat_multiply(_init_yaw_quat, ref_anchor_quat);
    std::vector<float> tgt_in_base = quat_multiply(quat_conjugate(robot_torso_quat), aligned_ref_anchor_quat);
    Eigen::Matrix3f mat = matrix_from_quat(tgt_in_base);
    std::vector<float> ref_anchor_ori_6d = {mat(0, 0), mat(0, 1), mat(1, 0), mat(1, 1), mat(2, 0), mat(2, 1)};

    // 安全逻辑 (重力投影)
    std::vector<float> proj_grav = QuatRotateInverse(robot_torso_quat, _gravity_vec);
    std::vector<float> motion_proj_grav = quat_apply_inverse(aligned_ref_anchor_quat, _gravity_vec);
    if (std::abs(motion_proj_grav[2] - proj_grav[2]) > _anchor_terminate_thresh)
        _terminate_flag = true;

    // 拼装 154 维观测
    _observation.clear();
    _observation.reserve(_obs_dim);
    _observation.insert(_observation.end(), ref_dof_pos.begin(), ref_dof_pos.end());             // 29
    _observation.insert(_observation.end(), ref_dof_vel.begin(), ref_dof_vel.end());             // 29
    _observation.insert(_observation.end(), ref_anchor_ori_6d.begin(), ref_anchor_ori_6d.end()); // 6
    _observation.insert(_observation.end(), body_ang_vel.begin(), body_ang_vel.end());           // 3
    _observation.insert(_observation.end(), dof_pos_rel.begin(), dof_pos_rel.end());             // 29
    _observation.insert(_observation.end(), dof_vel_rel.begin(), dof_vel_rel.end());             // 29
    _observation.insert(_observation.end(), _action.begin(), _action.end());                     // 29
    for (auto &v : _observation)
        v = std::clamp(v, -clip_observations, clip_observations);
}

void State_WBC_New::_action_compute()
{
    try
    {
        auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeCPU);
        std::vector<int64_t> shape = {1, _obs_size_};
        Ort::Value input = Ort::Value::CreateTensor<float>(mem, _observation.data(), _observation.size(), shape.data(), shape.size());
        auto outputs = _session->Run(Ort::RunOptions{nullptr}, _input_names.data(), &input, 1, _output_names.data(), 1);
        float *raw = outputs[0].GetTensorMutableData<float>();
        std::memcpy(_action.data(), raw, NUM_DOF * sizeof(float));

        // ==========================================
        // 重要：还原动作到物理电机位置
        // ==========================================
        for (int i = 0; i < NUM_DOF; i++)
        {

            float act_clamped = std::clamp(_action[i], -clip_actions, clip_actions);

            // ⚠️ 关键修正：必须使用网络索引 [i] 读取数组，然后赋值给 dof_mapping 映射出的物理电机！
            _joint_q[dof_mapping[i]] = act_clamped * _action_scale[i] + _default_dof_pos[i];
        }
    }
    catch (...)
    {
        std::cerr << "Inference error!" << std::endl;
    }
}

void State_WBC_New::enter()
{
    _pause_flag = false;
    _terminate_flag = false;
    _refer_idx = _start_refer_idx;
    if (_end_refer_idx < 0)
        _end_refer_idx = _motion_frame_count - 1;
    for (int i = 0; i < NUM_DOF; i++)
    {
        _lowCmd->motorCmd[i].mode = 10;
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
        _lowCmd->motorCmd[i].dq = 0;
        _lowCmd->motorCmd[i].tau = 0;
        _lowCmd->motorCmd[i].Kp = dof_Kps[i];
        _lowCmd->motorCmd[i].Kd = dof_Kds[i];
        _joint_q[i] = _lowState->motorState[i].q;
        _targetPos_rl[i] = _lowState->motorState[i].q;
        _last_targetPos_rl[i] = _lowState->motorState[i].q;
    }
    std::fill(_action.begin(), _action.end(), 0.0f);
    _init_yaw_quat = quat_multiply(yaw_quat(_current_torso_quat()), quat_conjugate(yaw_quat(_reference_torso_quat(_refer_idx))));
    _init_buffers();
}

void State_WBC_New::run()
{
    if (!_pause_flag)
        _refer_idx++;
    if (_refer_idx >= (unsigned int)_end_refer_idx)
        _refer_idx = _end_refer_idx;
    _observations_compute();
    _action_compute();
    for (int j = 0; j < NUM_DOF; j++)
    {
        _lowCmd->motorCmd[j].mode = 10; // ⚠️
        _lowCmd->motorCmd[j].q = _joint_q[j];
        _lowCmd->motorCmd[j].dq = 0;
        _lowCmd->motorCmd[j].tau = 0;
        _lowCmd->motorCmd[j].Kp = dof_Kps[j];
        _lowCmd->motorCmd[j].Kd = dof_Kds[j];
    }
}

void State_WBC_New::exit() { std::cout << "[State_WBC_New] Exit." << std::endl; }

FSMStateName State_WBC_New::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_B || _terminate_flag)
        return FSMStateName::PASSIVE;
    if (_lowState->userCmd == UserCommand::R2 && !_pause_flag)
    {
        _pause_flag = true;
        return FSMStateName::WBC;
    }
    if (_lowState->userCmd == UserCommand::R1 && _pause_flag)
    {
        _pause_flag = false;
        return FSMStateName::WBC;
    }
    return FSMStateName::WBC;
}
