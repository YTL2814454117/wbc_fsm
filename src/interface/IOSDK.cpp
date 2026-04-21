#include "interface/IOSDK.h"
#include <stdio.h>
#include <iostream>
// --- 新增头文件开始 ---
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
// --- 新增头文件结束 ---

// --- 新增：Linux 无阻塞键盘检测函数 ---
int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // 取消回车确认和按键回显
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

// 构造函数的实现
IOSDK::IOSDK()
{
    // ChannelFactory::Instance()->Init(0, "eth0"); // eth0 for real robot
    ChannelFactory::Instance()->Init(1, "lo"); // lo for simulation 强制绑定lo本地网卡

    // 创建了发布者和订阅者通道
    lowcmd_publisher_.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
    lowcmd_publisher_->InitChannel();

    lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
    lowstate_subscriber_->InitChannel(std::bind(&IOSDK::LowStateHandler, this, std::placeholders::_1), 1);

    // 计数器归零
    counter_ = 0;
    userCmd_ = UserCommand::NONE;
    userValue_.setZero();
    mode_machine_ = 0;

    // --- 新增：启动后台键盘监听线程 ---
    std::thread([this]()
                {
        while (true) {
            if (kbhit()) {
                char c = getchar();
                switch (c)
                {
                // ================= 核心状态切换 (数字键) =================
                case '0':
                    userCmd_ = UserCommand::SELECT;
                    std::cout << "\n[Key] EXIT (SELECT)" << std::endl;
                    break;
                case '1':
                    userCmd_ = UserCommand::START;
                    std::cout << "\n[Key] FIXED POSE (START)" << std::endl;
                    break;
                case '2':
                    userCmd_ = UserCommand::R2_A;
                    std::cout << "\n[Key] LOCO MODE (R2_A)" << std::endl;
                    break;
                case '3':
                    userCmd_ = UserCommand::R1_UP;
                    std::cout << "\n[Key] WBC MODE (R1_UP)" << std::endl;
                    break;
                case '4':
                    userCmd_ = UserCommand::R1_LEFT;
                    std::cout << "\n[Key] WBC LEFT (R1_LEFT)" << std::endl;
                    break;
                case '5':
                    userCmd_ = UserCommand::R1_RIGHT;
                    std::cout << "\n[Key] WBC RIGHT (R1_RIGHT)" << std::endl;
                    break;

                // ================= 运动控制与暂停 (功能键) =================
                case 'p':
                    userCmd_ = UserCommand::L2_B;
                    std::cout << "\n[Key] PASSIVE (L2_B)" << std::endl;
                    break;
                case '[':
                    userCmd_ = UserCommand::R2;
                    std::cout << "\n[Key] PAUSE IN SETTED IDX (R2)" << std::endl;
                    break;
                case ']':
                    userCmd_ = UserCommand::R1;
                    std::cout << "\n[Key] MOTION CONTINUE (R1)" << std::endl;
                    break;
                case 'l':
                    userCmd_ = UserCommand::L2;
                    std::cout << "\n[Key] PAUSE IN CURRENT IDX (L2)" << std::endl;
                    break;
                case 'b':
                    userCmd_ = UserCommand::R2_B;
                    std::cout << "\n[Key] BACK TO LOCO FROM AMP (R2_B)" << std::endl;
                    break;

                // ================= 速度档位调节 =================
                case '+':
                case '=':
                    userCmd_ = UserCommand::R2_UP;
                    std::cout << "\n[Key] HIGH SPEED MODE (R2_UP)" << std::endl;
                    break;
                case '-':
                    userCmd_ = UserCommand::R2_DOWN;
                    std::cout << "\n[Key] LOW SPEED MODE (R2_DOWN)" << std::endl;
                    break;

                    // ================= 摇杆速度控制 (WASD + QE) =================
                    // ================= 纯净版摇杆控制 =================
                case 'w':
                    userValue_.ly = 0.5; // 前进
                    userValue_.lx = 0.0; // 强制清零侧移
                    userValue_.rx = 0.0; // 强制清零转向
                    userValue_.ry = 0.0; // 保持不变
                    std::cout << "\n[Key] Forward (Pure)" << std::endl;
                    break;
                case 's':
                    userValue_.ly = -0.5;
                    userValue_.lx = 0.0;
                    userValue_.rx = 0.0;
                    userValue_.ry = 0.0;
                    std::cout << "\n[Key] Backward (Pure)" << std::endl;
                    break;
                case 'a':
                    userValue_.lx = 0.0;
                    userValue_.ly = 0.0;
                    userValue_.rx = 0.5;
                    userValue_.ry = 0.0;
                    std::cout << "\n[Key] Left Strafing" << std::endl;
                    break;
                case 'd':
                    userValue_.lx = 0.0;
                    userValue_.ly = 0.0;
                    userValue_.rx = -0.5;
                    userValue_.ry = 0.0;
                    std::cout << "\n[Key] Right Strafing" << std::endl;
                    break;
                // 保留单独的转向键，用来手动纠偏
                case 'q':
                    userValue_.rx = 0.5;
                    std::cout << "\n[Key] Turn Left" << std::endl;
                    break;
                case 'e':
                    userValue_.rx = -0.5;
                    std::cout << "\n[Key] Turn Right" << std::endl;
                    break;

                // ================= 紧急刹车 =================
                case ' ':
                    userValue_.lx = 0;
                    userValue_.ly = 0;
                    userValue_.rx = 0;
                    std::cout << "\n[Key] STOP!" << std::endl;
                    break;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 休息20ms，防止吃光CPU
        } })
        .detach(); // 让这个线程在后台默默运行
    // --- 新增结束 ---
}

// 发送接收函数的实现
void IOSDK::sendRecv(const LowlevelCmd *cmd, LowlevelState *state)
{
    // send control cmd
    LowCmd_ dds_low_command;                                    // dds是宇树内部通信的消息格式，LowCmd_是发送给底层的控制命令消息类型
    dds_low_command.mode_pr() = static_cast<uint8_t>(Mode::PR); // 按照底层控制模式要求设置mode_pr字段
    dds_low_command.mode_machine() = mode_machine_;
    for (size_t i = 0; i < G1_NUM_MOTOR; i++) // 遍历29个电机，将控制命令中的数据填充到dds_low_command的motor_cmd字段中
    {

        dds_low_command.motor_cmd().at(i).mode() = 1;                   // 1:Enable, 0:Disable
        dds_low_command.motor_cmd().at(i).tau() = cmd->motorCmd[i].tau; // 电机力矩
        dds_low_command.motor_cmd().at(i).q() = cmd->motorCmd[i].q;     // 关节位置
        dds_low_command.motor_cmd().at(i).dq() = cmd->motorCmd[i].dq;   // 关节速度
        dds_low_command.motor_cmd().at(i).kp() = cmd->motorCmd[i].Kp;   // 位置环刚度
        dds_low_command.motor_cmd().at(i).kd() = cmd->motorCmd[i].Kd;   // 微分系数
        // std::cout<<"des_q: "<<dds_low_command.motor_cmd().at(i).q()<<std::endl;
    }

    dds_low_command.crc() = crc32_core((uint32_t *)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1); // 计算CRC校验码，并填充到dds_low_command的crc字段中
    bool wrt = lowcmd_publisher_->Write(dds_low_command);                                                 // 将控制命令发布到DDS通道，发送给底层控制器

    for (int i = 0; i < G1_NUM_MOTOR; i++)
    {
        state->motorState[i].q = _lowState.motorState[i].q; // 从接收到的_lowState中读取电机状态数据，填充到输出参数state中，以供上层状态机使用
        state->motorState[i].dq = _lowState.motorState[i].dq;
    }
    for (int i = 0; i < 3; i++)
    {
        state->imu.quaternion[i] = _lowState.imu.quaternion[i]; // 填充IMU状态数据
        state->imu.accelerometer[i] = _lowState.imu.accelerometer[i];
        state->imu.gyroscope[i] = _lowState.imu.gyroscope[i];
    }
    state->imu.quaternion[3] = _lowState.imu.quaternion[3];

    state->userCmd = userCmd_; // 键盘或者手柄输入的用户命令
    state->userValue = userValue_;
}

void IOSDK::LowStateHandler(const void *message)
{
    LowState_ low_state = *(const LowState_ *)message;
    if (low_state.crc() != crc32_core((uint32_t *)&low_state, (sizeof(LowState_) >> 2) - 1))
    {
        std::cout << "[ERROR] CRC Error" << std::endl;
        return;
    }

    // get motor state
    for (int i = 0; i < G1_NUM_MOTOR; ++i)
    {
        _lowState.motorState[i].q = low_state.motor_state()[i].q();
        _lowState.motorState[i].dq = low_state.motor_state()[i].dq();
    }

    // get imu state
    _lowState.imu.gyroscope[0] = low_state.imu_state().gyroscope()[0];
    _lowState.imu.gyroscope[1] = low_state.imu_state().gyroscope()[1];
    _lowState.imu.gyroscope[2] = low_state.imu_state().gyroscope()[2];

    _lowState.imu.quaternion[0] = low_state.imu_state().quaternion()[0];
    _lowState.imu.quaternion[1] = low_state.imu_state().quaternion()[1];
    _lowState.imu.quaternion[2] = low_state.imu_state().quaternion()[2];
    _lowState.imu.quaternion[3] = low_state.imu_state().quaternion()[3];

    _lowState.imu.accelerometer[0] = low_state.imu_state().accelerometer()[0];
    _lowState.imu.accelerometer[1] = low_state.imu_state().accelerometer()[1];
    _lowState.imu.accelerometer[2] = low_state.imu_state().accelerometer()[2];

    // update gamepad
    memcpy(rx_.buff, &low_state.wireless_remote()[0], 40);
    gamepad_.update(rx_.RF_RX);

    // update mode machine
    if (mode_machine_ != low_state.mode_machine())
    {
        if (mode_machine_ == 0)
            std::cout << "G1 type: " << unsigned(low_state.mode_machine()) << std::endl;
        mode_machine_ = low_state.mode_machine();
    }

    /*
    if (gamepad_.start.pressed)
    {
        userCmd_ = UserCommand::START;
    }
    if (gamepad_.select.pressed)
    {
        userCmd_ = UserCommand::SELECT;
    }

    if (gamepad_.R2.pressed)
    {
        userCmd_ = UserCommand::R2;
    }
    if (gamepad_.L2.pressed)
    {
        userCmd_ = UserCommand::L2;
    }
    if (gamepad_.R1.pressed)
    {
        userCmd_ = UserCommand::R1;
    }
    if (gamepad_.R2.pressed && gamepad_.A.pressed)
    {
        userCmd_ = UserCommand::R2_A;
    }
    if (gamepad_.L2.pressed && gamepad_.B.pressed)
    {
        userCmd_ = UserCommand::L2_B;
    }
    if (gamepad_.R1.pressed && gamepad_.up.pressed)
    {
        userCmd_ = UserCommand::R1_UP;
    }
    if (gamepad_.R1.pressed && gamepad_.left.pressed)
    {
        userCmd_ = UserCommand::R1_LEFT;
    }
    if (gamepad_.R1.pressed && gamepad_.right.pressed)
    {
        userCmd_ = UserCommand::R1_RIGHT;
    }
    if (gamepad_.R2.pressed && gamepad_.up.pressed)
    {
        userCmd_ = UserCommand::R2_UP;
    }
    if (gamepad_.R2.pressed && gamepad_.down.pressed)
    {
        userCmd_ = UserCommand::R2_DOWN;
    }
    if (gamepad_.R2.pressed && gamepad_.B.pressed)
    {
        userCmd_ = UserCommand::R2_B;
    }

    userValue_.lx = -gamepad_.lx;
    userValue_.ly = gamepad_.ly;
    userValue_.rx = -gamepad_.rx;
    userValue_.ry = gamepad_.ry;
    */
}