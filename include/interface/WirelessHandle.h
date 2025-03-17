/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef WIRELESSHANDLE_H
#define WIRELESSHANDLE_H

#include "message/unitree_joystick.h"
#include "interface/CmdPanel.h"
#include <unitree/idl/go2/LowState_.hpp>

class WirelessHandle : public CmdPanel{
public:
    WirelessHandle();
    ~WirelessHandle(){}
    void receiveHandle(unitree_go::msg::dds_::LowState_ *lowState);
private:
    xRockerBtnDataStruct _keyData;
};

#endif  // WIRELESSHANDLE_H