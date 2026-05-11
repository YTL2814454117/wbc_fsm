#ifndef CTRLCOMPONENTS_H
#define CTRLCOMPONENTS_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/IOInterface.h"
#include "interface/CmdPanel.h"
#include "common/DancePolicyManager.h"
#include "common/RuntimePaths.h"
#include <string>
#include <iostream>


struct CtrlComponents{
public:
    CtrlComponents(IOInterface *ioInter):ioInter(ioInter){
        lowCmd = new LowlevelCmd();
        lowState = new LowlevelState();
        danceManager = new DancePolicyManager(
            RuntimePaths::resolve("config/wbc_dances.json"),
            RuntimePaths::root());
        exitFlag = false;
    }
    ~CtrlComponents(){
        delete lowCmd;
        delete lowState;
        delete danceManager;
        delete ioInter;
    }
    LowlevelCmd *lowCmd;
    LowlevelState *lowState;
    IOInterface *ioInter;
    DancePolicyManager *danceManager;

    double dt;
    bool *running;
    bool exitFlag;
    CtrlPlatform ctrlPlatform;

    void sendRecv(){
        ioInter->sendRecv(lowCmd, lowState);  
    }



};

#endif  // CTRLCOMPONENTS_H
