#pragma once

#include <RoboClaw.h>
#include "pid.h"
#include "taskhandler.h"
#include "motor.h"
#include "controller.h"

typedef struct{
  int read_pin;  
  float offset; 
  float scale; 
} loadcell_t; 

typedef enum{
  STATE_NOT_HOMED,
  STATE_START_HOME,
  STATE_WAITING_ENC1,
  STATE_WAITING_ENC2,
  STATE_MEASURE_YGND,
  STATE_MOVE_TO_OFFSET, 
  STATE_PRE_DUMP,
  STATE_PAUSE_1,
  STATE_PAUSE1_COMPLETE,
  STATE_PAUSE_2,
  STATE_PAUSE2_COMPLETE,
  STATE_HOME_COMPLETE
} homing_state_t; 

typedef enum{
  WAITING_WAYPOINT,
  RECEIVED_WAYPOINT  
} waypoint_state_t; 

class scoop {

private:
//Pinout
  RoboClaw &roboclaw;
  uint8_t address;
  Controller *controller; 

  int xLimPin;
  int yLimPin;
  int motor1dir;
  int motor2dir;

//Joint Limits:
  float xMax;
  float xMin;
  float yMax;
  float yMin;
  float pitchMax;
  float pitchMin; 
  float vibeMin;
  float vibeMax; 

//Homing Parameters
  uint32_t M1_lim_counts; 
  uint32_t M2_lim_counts;
  float y_gnd;  

//Constants
  static constexpr float gearLead = 8.0f;
  static constexpr float encoder_output_ratio = 534.7 *1.0;
  static constexpr float x0 = 10.0f;
  static constexpr float y0 = 10.0f;
  static constexpr float y_gnd_pre = 50.0f;
  static constexpr float grav = 9.81f;
  static constexpr uint32_t homing_speed = 1000;



//Control
  command_t activeCommand; 
  homing_state_t home_status; 
  waypoint_state_t waypoint_state; 
  static scoop* instance; 
  float home_pause_time; 


public:

  waypoint_t *current_waypoint; 
  motor *pitchMotor; 
  motor *vibeMotor; 
  
//Load Cells
  loadcell_t *fx_cell; 
  loadcell_t *fy_cell; 

  //Contructor
  scoop(RoboClaw &roboclaw, uint8_t address, 
          Controller *controller,
          int xLimPin, int yLimPin, 
          int x_motor_dir, int y_motor_dir, 
          motor *pitchMotor, motor *vibeMotor,
          loadcell_t *fx_cell, loadcell_t *fy_cell, 
          float xMax, float xMin, 
          float yMax, float yMin, 
          float pitchMax, float pitchMin, 
          float vibeMin, float vibeMax);

    void move_task(); 
    void move_waypoint(waypoint_t *waypoint, uint32_t v_max = 5000, uint32_t a_max = 10000); 
    void PID_task();
    void home_task(); 
    void waypoint_task(); 
    void commandHandlerTask(); 
    static void process_command(uint8_t *buffer); 
    void update_position(waypoint_t *waypoint);
    void set_command(command_t command); 
    void home();
    void excavateSequence(); 
    void readAccel();
    void read_force(); 
    void readAllSensors();
    void displayspeed();
    void inputControl(int analogX, int analogY, int analogPitch, int analogVibe, float deadBand = 30, float stepSize = 0.5);
    void init(); 
    void rampVibeSpeed(); 
    uint32_t trans2rot(float value);
    uint32_t rot2trans(float value); 
};


