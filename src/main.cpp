#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cstdlib>
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
#include "common/RuntimePaths.h"

#if ENABLE_QIANER_LICENSE_AUTH
#include "auth/CloudActivator.hpp"
#include <nlohmann/json.hpp>
#endif

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

#if ENABLE_QIANER_LICENSE_AUTH
struct QianerAuthConfig
{
    std::string cert_path = RuntimePaths::resolve("../qianer_auth_project/keys/ZJUDES.crt");
    std::string license_path = RuntimePaths::resolve("license/qianer_license.lic");
    std::string iface = "eth0";
};

std::string getEnvOrDefault(const char *name, const std::string &default_value)
{
    const char *value = std::getenv(name);
    if (value == nullptr || std::string(value).empty())
        return default_value;
    return std::string(value);
}

QianerAuthConfig loadQianerAuthConfig()
{
    QianerAuthConfig config;
    const std::string config_path = RuntimePaths::resolve("config/qianer_auth.json");
    std::ifstream file(config_path);
    if (file.is_open())
    {
        try
        {
            nlohmann::json j;
            file >> j;
            if (j.contains("cert_path") && j["cert_path"].is_string())
                config.cert_path = RuntimePaths::resolve(j["cert_path"].get<std::string>());
            if (j.contains("license_path") && j["license_path"].is_string())
                config.license_path = RuntimePaths::resolve(j["license_path"].get<std::string>());
            if (j.contains("iface") && j["iface"].is_string())
                config.iface = j["iface"].get<std::string>();
        }
        catch (const std::exception &e)
        {
            std::cerr << "[QianerAuth] Failed to parse " << config_path
                      << ": " << e.what() << ". Using built-in defaults." << std::endl;
        }
    }
    else
    {
        std::cout << "[QianerAuth] Config file not found: " << config_path
                  << ". Using built-in defaults." << std::endl;
    }

    config.cert_path = RuntimePaths::resolve(getEnvOrDefault("QIANER_AUTH_CERT_PATH", config.cert_path));
    config.license_path = RuntimePaths::resolve(getEnvOrDefault("QIANER_AUTH_LICENSE_PATH", config.license_path));
    config.iface = getEnvOrDefault("QIANER_AUTH_IFACE", config.iface);
    return config;
}

bool verifyQianerLicense()
{
    const QianerAuthConfig config = loadQianerAuthConfig();

    std::cout << "[QianerAuth] License verification is enabled." << std::endl;
    std::cout << "[QianerAuth] cert=" << config.cert_path
              << " license=" << config.license_path
              << " iface=" << config.iface << std::endl;

    CloudActivator activator("", config.cert_path, config.license_path);
    if (!activator.verifyLocalLicense(config.iface))
    {
        std::cerr << "[QianerAuth] License verification failed. Controller startup blocked." << std::endl;
        return false;
    }

    std::cout << "[QianerAuth] License verification passed." << std::endl;
    return true;
}
#endif

int main(int argc, char **argv)
{

    setProcessScheduler();
    std::cout << std::fixed << std::setprecision(3); // 设置终端打印浮点数的精度为小数点后3位

#if ENABLE_QIANER_LICENSE_AUTH
    if (!verifyQianerLicense())
    {
        return 1;
    }
#else
    std::cout << "[QianerAuth] License verification is disabled by build macro." << std::endl;
#endif

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
