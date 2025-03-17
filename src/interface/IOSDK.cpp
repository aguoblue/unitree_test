/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_REAL_ROBOT

#include "interface/IOSDK.h"
#include "interface/WirelessHandle.h"
#include "interface/KeyBoard.h"
#include <stdio.h>

IOSDK::IOSDK(const std::string& networkInterface) : _networkInterface(networkInterface) {
    std::cout << "IOSDK lo" << std::endl;

    // 初始化 ChannelFactory
    ChannelFactory::Instance()->Init(1, _networkInterface.c_str());

    // 初始化 DDS 通道
    initChannels();

    // 初始化无线手柄
    // cmdPanel = new WirelessHandle();

    //初始化键盘
    cmdPanel = new KeyBoard();

    // 启动发送线程
    _lowCmdWriteThread = CreateRecurrentThreadEx("writeLowCmd", UT_CPU_ID_NONE, 2000, &IOSDK::lowCmdWrite, this);
}

void IOSDK::initChannels() {
    std::cout << "IOSDK initChannels" << std::endl;
    // 初始化 LowCmd
    _lowCmd.head()[0] = 0xFE;
    _lowCmd.head()[1] = 0xEF;
    _lowCmd.level_flag() = 0xFF;
    _lowCmd.gpio() = 0;
    for (int i = 0; i < 20; i++) {
        _lowCmd.motor_cmd()[i].mode() = 0x01; // 伺服模式
        _lowCmd.motor_cmd()[i].q() = 2.146E+9f; // PosStopF
        _lowCmd.motor_cmd()[i].kp() = 0;
        _lowCmd.motor_cmd()[i].dq() = 16000.0f; // VelStopF
        _lowCmd.motor_cmd()[i].kd() = 0;
        _lowCmd.motor_cmd()[i].tau() = 0;
    }

    // 创建发布者
    _lowCmdPublisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    _lowCmdPublisher->InitChannel();

    // 创建订阅者
    _lowStateSubscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    _lowStateSubscriber->InitChannel(std::bind(&IOSDK::lowStateHandler, this, std::placeholders::_1), 1);
}

void IOSDK::lowStateHandler(const void* message) {
    _lowState = *(unitree_go::msg::dds_::LowState_*)message;
}

void IOSDK::lowCmdWrite() {
    // 计算 CRC 并发布命令
    _lowCmd.crc() = crc32_core((uint32_t*)&_lowCmd, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    _lowCmdPublisher->Write(_lowCmd);
}

uint32_t IOSDK::crc32_core(uint32_t* ptr, uint32_t len) {
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++) {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            } else {
                CRC32 <<= 1;
            }
            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }
    return CRC32;
}

void IOSDK::sendRecv(const LowlevelCmd *cmd, LowlevelState *state) {
    // 将输入的 LowlevelCmd 转换为 unitree_sdk2 的 LowCmd
    for (int i = 0; i < 12; ++i) {
        _lowCmd.motor_cmd()[i].mode() = cmd->motorCmd[i].mode;
        _lowCmd.motor_cmd()[i].q() = cmd->motorCmd[i].q;
        _lowCmd.motor_cmd()[i].dq() = cmd->motorCmd[i].dq;
        _lowCmd.motor_cmd()[i].kp() = cmd->motorCmd[i].Kp;
        _lowCmd.motor_cmd()[i].kd() = cmd->motorCmd[i].Kd;
        _lowCmd.motor_cmd()[i].tau() = cmd->motorCmd[i].tau;
    }

    // 发送命令（由线程处理，这里仅更新 _lowCmd）

    // 将接收到的 LowState 转换为输出状态
    for (int i = 0; i < 12; ++i) {
        state->motorState[i].q = _lowState.motor_state()[i].q();
        state->motorState[i].dq = _lowState.motor_state()[i].dq();
        state->motorState[i].ddq = _lowState.motor_state()[i].ddq();
        state->motorState[i].tauEst = _lowState.motor_state()[i].tau_est();
        state->motorState[i].mode = _lowState.motor_state()[i].mode();
    }

    // IMU 数据
    for (int i = 0; i < 3; ++i) {
        state->imu.quaternion[i] = _lowState.imu_state().quaternion()[i];
        state->imu.gyroscope[i] = _lowState.imu_state().gyroscope()[i];
        state->imu.accelerometer[i] = _lowState.imu_state().accelerometer()[i];
    }
    state->imu.quaternion[3] = _lowState.imu_state().quaternion()[3];

    // 处理无线手柄（保持原有逻辑）
    cmdPanel->receiveHandle(&_lowState);
    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
}

// void IOSDK::sendRecv(const LowlevelCmd *cmd, LowlevelState *state){
//     for(int i(0); i < 12; ++i){
//         _lowCmd.motorCmd[i].mode = cmd->motorCmd[i].mode;
//         _lowCmd.motorCmd[i].q    = cmd->motorCmd[i].q;
//         _lowCmd.motorCmd[i].dq   = cmd->motorCmd[i].dq;
//         _lowCmd.motorCmd[i].Kp   = cmd->motorCmd[i].Kp;
//         _lowCmd.motorCmd[i].Kd   = cmd->motorCmd[i].Kd;
//         _lowCmd.motorCmd[i].tau  = cmd->motorCmd[i].tau;
//     }
    
//     _udp.SetSend(_lowCmd);
//     _udp.Send();

//     _udp.Recv();
//     _udp.GetRecv(_lowState);

//     for(int i(0); i < 12; ++i){
//         state->motorState[i].q = _lowState.motorState[i].q;
//         state->motorState[i].dq = _lowState.motorState[i].dq;
//         state->motorState[i].ddq = _lowState.motorState[i].ddq;
//         state->motorState[i].tauEst = _lowState.motorState[i].tauEst;
//         state->motorState[i].mode = _lowState.motorState[i].mode;
//     }

//     for(int i(0); i < 3; ++i){
//         state->imu.quaternion[i] = _lowState.imu.quaternion[i];
//         state->imu.gyroscope[i]  = _lowState.imu.gyroscope[i];
//         state->imu.accelerometer[i] = _lowState.imu.accelerometer[i];
//     }
//     state->imu.quaternion[3] = _lowState.imu.quaternion[3];

//     cmdPanel->receiveHandle(&_lowState);
//     state->userCmd = cmdPanel->getUserCmd();
//     state->userValue = cmdPanel->getUserValue();
// }



#endif  // COMPILE_WITH_REAL_ROBOT