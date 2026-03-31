#ifndef IOINTERFACE_H
#define IOINTERFACE_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/CmdPanel.h"
#include <string>

// 抽象接口类
class IOInterface
{
public:
    IOInterface() {}                                                         // 构造函数
    ~IOInterface() { delete cmdPanel; }                                      // 机构函数
    virtual void sendRecv(const LowlevelCmd *cmd, LowlevelState *state) = 0; // 纯虚函数，传入底层指令和底层状态
    void zeroCmdPanel() { cmdPanel->setZero(); }                             // 指令初始化
    void setPassive() { cmdPanel->setPassive(); }                            // 设置为被动阻尼状态

protected:
    CmdPanel *cmdPanel; // 传家宝，只有父类和子类可以调用
};

#endif // IOINTERFACE_H