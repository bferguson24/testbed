#pragma once

#include <Arduino.h>
#include "scoop.h"


// typedef __attribute__((packed))uint8_t command_t;
// enum {
//   COMMAND_START,
//   COMMAND_STOP,
//   COMMAND_HOME,
//   COMMAND_MOVE_WAYPOINT,
//   COMMAND_MOVE_MANUAL,
//   COMMAND_IDLE
// }; 

typedef enum command_t : uint8_t {
  COMMAND_IDLE = 0,
  COMMAND_START = 1,
  COMMAND_STOP = 2,
  COMMAND_HOME = 3,
  COMMAND_MOVE_WAYPOINT = 4,
  COMMAND_MOVE_MANUAL = 5
} command_t;




typedef __attribute__((packed)) struct{
  float x;
  float y;
  float pitch; 
  float vibe_speed; 
} waypoint_t; 

typedef __attribute__((packed)) struct{
  command_t cmd;
  waypoint_t waypoint; 
} move_cmd_t;
 