#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <iomanip>
#include <vector>
#include <cstring>
#include <openssl/sha.h>
#include <openssl/rsa.h>
#include <openssl/pem.h>
#include <openssl/bio.h>
#include <openssl/err.h>
#include <openssl/evp.h>
#include <openssl/buffer.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "control/ControlFrame.h"
#include "control/CtrlComponents.h"
#include "interface/IOSDK.h"

bool running = true;

void ShutDown(int sig)
{
    std::cout << "stop the controller" << std::endl;
    running = false;
}

void setProcessScheduler() // 进程实时调度设置
{
    pid_t pid = getpid(); // 获取程序的进程号
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO); // 设置为最高优先级
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)     // 向操作系统注册调度策略和优先级
    {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

int main(int argc, char **argv)
{

    setProcessScheduler();
    std::cout << std::fixed << std::setprecision(3); // 设置终端打印浮点数的精度为小数点后3位
    IOInterface *ioInter;                            // 接口类
    CtrlPlatform ctrlPlat;                           // 定义控制平台

    ioInter = new IOSDK();              // 接口的实现
    ctrlPlat = CtrlPlatform::REALROBOT; // API级别硬件仿真

    CtrlComponents *ctrlComp = new CtrlComponents(ioInter); // 实现接口类的控制组件
    ctrlComp->ctrlPlatform = ctrlPlat;
    ctrlComp->dt = 0.02; // 控制周期为20ms
    ctrlComp->running = &running;

    ControlFrame ctrlFrame(ctrlComp); // 控制框架，包含状态机和控制组件
    signal(SIGINT, ShutDown);         // 注册信号处理函数，当接收到SIGINT信号时调用ShutDown函数

    while (running)
    {
        if (ctrlComp->exitFlag)
            break;
        ctrlFrame.run();
    }

    delete ctrlComp; // 释放控制组件资源
    return 0;
}
