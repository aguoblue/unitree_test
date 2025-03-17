/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef IOSDK_H
#define IOSDK_H

#include "interface/IOInterface.h"
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/common/thread/thread.hpp>

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

class IOSDK : public IOInterface{
public:
IOSDK(const std::string& networkInterface = "lo"); // 默认网络接口
~IOSDK() {}
void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);

private:
    void initChannels();              // 初始化 DDS 通道
    void lowStateHandler(const void* message); // 处理接收到的状态
    void lowCmdWrite();               // 发送命令的线程函数
    uint32_t crc32_core(uint32_t* ptr, uint32_t len); // CRC 计算函数

private:
    unitree_go::msg::dds_::LowCmd_ _lowCmd{};    // 新 SDK 的命令
    unitree_go::msg::dds_::LowState_ _lowState{}; // 新 SDK 的状态

    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> _lowCmdPublisher;    // 发布者
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> _lowStateSubscriber; // 订阅者

    ThreadPtr _lowCmdWriteThread;     // 发送线程
    std::string _networkInterface;    // 网络接口
    bool _firstRun = true;            // 初始化标志

};

#endif  // IOSDK_H