#ifndef __ROS_INTERFACE_H__
#define __ROS_INTERFACE_H__

#include "Arduino.h"

typedef enum {
  ROS_LINK_DISABLED = 0,  // built without src/ros_config.h
  ROS_LINK_WIFI_WAIT,
  ROS_LINK_AGENT_WAIT,
  ROS_LINK_CONNECTED,
} RosLinkState_t;

// Starts the micro-ROS task on core 0, next to the display task. Control stays
// alone on core 1; nothing here touches I2C.
void RosTaskStart();

RosLinkState_t RosGetLinkState();

#endif
