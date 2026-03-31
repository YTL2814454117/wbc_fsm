
#ifndef CONTROLFRAME_H
#define CONTROLFRAME_H

#include "FSM/FSM.h"
#include "control/CtrlComponents.h"

// 控制框架类，包含状态机和控制组件
class ControlFrame
{
public:
	ControlFrame(CtrlComponents *ctrlComp);
	~ControlFrame()
	{
		delete _FSMController;
	}
	void run(); // 声明控制框架的运行函数

private:
	FSM *_FSMController;
	CtrlComponents *_ctrlComp;
};

#endif // CONTROLFRAME_H