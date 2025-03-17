/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_REAL_ROBOT

#include "interface/IOSDK.h"
#include "interface/WirelessHandle.h"
#include <stdio.h>

#ifdef ROBOT_TYPE_Go1
IOSDK::IOSDK():_safe(UNITREE_LEGGED_SDK::LeggedType::Aliengo), _udp(UNITREE_LEGGED_SDK::LOWLEVEL, 8090, "192.168.123.10", 8007){
    std::cout << "The control interface for real robot" << std::endl;
    _udp.InitCmdData(_lowCmd);
    cmdPanel = new WirelessHandle();
}
#endif

#ifdef ROBOT_TYPE_A1
IOSDK::IOSDK():_safe(UNITREE_LEGGED_SDK::LeggedType::Aliengo), _udp(UNITREE_LEGGED_SDK::LOWLEVEL){
    std::cout << "The control interface for real robot" << std::endl;
    _udp.InitCmdData(_lowCmd);
    cmdPanel = new WirelessHandle();

}
#endif


void IOSDK::sendRecv(const LowlevelCmd *cmd, LowlevelState *state){
    for(int i(0); i < 12; ++i){
        _lowCmd.motorCmd[i].mode = cmd->motorCmd[i].mode;
        _lowCmd.motorCmd[i].q    = cmd->motorCmd[i].q;
        _lowCmd.motorCmd[i].dq   = cmd->motorCmd[i].dq;
        _lowCmd.motorCmd[i].Kp   = cmd->motorCmd[i].Kp;
        _lowCmd.motorCmd[i].Kd   = cmd->motorCmd[i].Kd;
        _lowCmd.motorCmd[i].tau  = cmd->motorCmd[i].tau;
    }
    
    _udp.SetSend(_lowCmd);
    _udp.Send();

    _udp.Recv();
    _udp.GetRecv(_lowState);

    for(int i(0); i < 12; ++i){
        state->motorState[i].q = _lowState.motorState[i].q;
        state->motorState[i].dq = _lowState.motorState[i].dq;
        state->motorState[i].ddq = _lowState.motorState[i].ddq;
        state->motorState[i].tauEst = _lowState.motorState[i].tauEst;
        state->motorState[i].mode = _lowState.motorState[i].mode;
    }

    for(int i(0); i < 3; ++i){
        state->imu.quaternion[i] = _lowState.imu.quaternion[i];
        state->imu.gyroscope[i]  = _lowState.imu.gyroscope[i];
        state->imu.accelerometer[i] = _lowState.imu.accelerometer[i];
    }
    state->imu.quaternion[3] = _lowState.imu.quaternion[3];

    cmdPanel->receiveHandle(&_lowState);
    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
}

#endif  // COMPILE_WITH_REAL_ROBOT