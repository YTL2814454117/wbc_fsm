#ifndef WBC_NEW_H
#define WBC_NEW_H

#include "FSM/FSMState.h"
#include "common/read_traj.h"
#include "common/mathTools.h"
#include <onnxruntime_cxx_api.h>
#include <cstring>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <deque>
#include <cmath>
#include <stdexcept>

#define NUM_DOF 29

using namespace ArmatureConstants;

class State_WBC_New : public FSMState
{
public:
    State_WBC_New(CtrlComponents *ctrlComp);
    ~State_WBC_New() = default;
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

private:
    Ort::Env _env;
    Ort::SessionOptions _session_options;
    std::unique_ptr<Ort::Session> _session;
    Ort::AllocatorWithDefaultOptions _allocator;

    const std::vector<const char *> _input_names = {"obs"};
    const std::vector<const char *> _output_names = {"actions"};

    int64_t _obs_size_;
    int64_t _action_size_;

    float _targetPos_rl[NUM_DOF];
    float _last_targetPos_rl[NUM_DOF];

    void _loadPolicy();
    void _observations_compute();
    void _action_compute();

    const float clip_observations = 100.0;
    const float clip_actions = 100.0;

    const float scale_lin_vel = 1.0;
    const float scale_ang_vel = 1.0;
    float scale_dof_pos = 1.0;
    float scale_dof_vel = 1.0;
    float _joint_q[NUM_DOF];

    std::vector<float> _action;      // 上一步的动作向量
    std::vector<float> _observation; // 154维观测向量

    void _init_buffers();

    // 动作捕捉/参考轨迹数据
    std::vector<float> _body_ang_vel_w;
    std::vector<uint32_t> _body_ang_vel_w_shape;
    std::vector<float> _body_pos_w;
    std::vector<uint32_t> _body_pos_w_shape;
    std::vector<float> _body_quat_w;
    std::vector<uint32_t> _body_quat_w_shape;
    std::vector<float> _joint_pos;
    std::vector<uint32_t> _joint_pos_shape;
    std::vector<float> _joint_vel;
    std::vector<uint32_t> _joint_vel_shape;

    bool _bin_data_loaded;
    std::string _model_path;
    std::string _folder_path;

    const int _obs_dim = 154;
    unsigned int _refer_idx = 0;
    unsigned int _last_refer_idx = 0;
    const int _anchor_idx = 0;
    bool _pause_flag = false;
    int _start_refer_idx = 0;
    int _pause_refer_idx = 350;
    int _end_refer_idx = -1;
    int _motion_frame_count = 0;
    const std::vector<float> _gravity_vec = {0.0f, 0.0f, -1.0f};
    float _anchor_terminate_thresh = 0.5f;
    bool _terminate_flag = false;
    bool _pause_curr_flag = false;

    // --- 以下参数严格对应你提供的配置文件 ---

    // 1. 关节 ID 映射 (模型索引 -> 电机物理ID)
    const int dof_mapping[NUM_DOF] = {
        0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10, 16, 23, 5, 11,
        17, 24, 18, 25, 19, 26, 20, 27, 21, 28};

    // 2. 默认关节位置 (Offset / Default Pose) - 对应模型输出顺序
    const float _default_dof_pos[NUM_DOF] = {
        -0.302, -0.319, 0.00124, 0.000442, 0.00489, 0.00191, 0.00929, 0.00796,
        0.00546, 0.672, 0.67, 0.2, 0.202, -0.368, -0.355, 0.194, -0.196, -0.00644, 0.00976,
        0.00258, -0.00029, 0.605, 0.596, 0.00818, 0.00322, 0.00293, -0.00339, -0.00955,
        -0.00715};

    // 3. 动作缩放因子 (Action Scale) - 对应模型输出顺序
    const float _action_scale[NUM_DOF] = {
        0.548, 0.548, 0.548, 0.351, 0.351, 0.439, 0.548, 0.548, 0.439, 0.351,
        0.351, 0.439, 0.439, 0.439, 0.439, 0.439, 0.439, 0.439, 0.439, 0.439,
        0.439, 0.439, 0.439, 0.439, 0.439, 0.0745, 0.0745, 0.0745, 0.0745};

    // 4. 电机 PD 增益 (按物理电机 ID 0-28 存储)
    double dof_Kps[NUM_DOF];
    double dof_Kds[NUM_DOF];
};

#endif // WBC_NEW_H