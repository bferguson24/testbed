// #include "motor.h"
#include <RoboClaw.h>
#include <SoftwareSerial.h>
#include "scoop.h"
#include "HX711.h"
#include "utility.h"
#include "taskhandler.h"
#include "packet.h"
#include "controller.h"
#include "motor.h"
#include "pid.h"

//Packet 
SerialPacket packet; 
Controller controller(30, 0.5); 

//Scoop Parameters
  #define address 0x80  
  const int xLimPin = 6;
  const int yLimPin = 7; 
  const int x_motor_dir = 1;
  const int y_motor_dir = -1; 

//Pitch motor:
  int pitch_dir1 = 13;
  int pitch_dir2 = 12;
  int pitch_output_direction_invert = 1; 
  int pitch_angle_read_invert = 1;  
  int pitch_pwm = 2;
  int pitch_angle_pin = A7; 
  int pitch_angle_offset = 670;
  sample pitch_avg(0.05, 0.0); 
  PID_controller pitch_PID (3.0,0.0,1.0, -100, 100, 0.0, 3.0); 
  motor pitch_motor(pitch_pwm, 
                    pitch_dir1, 
                    pitch_dir2, 
                    pitch_angle_pin, 
                    pitch_angle_offset, 
                    pitch_output_direction_invert, 
                    pitch_angle_read_invert, 
                    &pitch_PID, 
                    &pitch_avg
                  );

//Vibe Motor
  int vibe_dir1 = 10;
  int vibe_dir2 = 11;
  int vibe_pwm = 3;
  motor vibe_motor(vibe_pwm, vibe_dir1, vibe_dir2); 
  
//Joint Limits:
  float xMax = 700; //[mm]
  float xMin = 1; 
  float yMax = 190; 
  float yMin = 1; 
  float pitchMax = 30; // [Deg]
  float pitchMin = -30; 
  float vibeMin = 0;
  float vibeMax = 100; 

//Load Cells
  int fx_pin = A5;
  float fx_offset = 0.0f;
  float fx_scale = 0.0f; 
  loadcell_t fx_cell = {fx_pin, fx_offset, fx_scale};

  int fy_pin = A6;
  float fy_offset = 0.0f;
  float fy_scale = 0.0f; 
  loadcell_t fy_cell = {fy_pin, fy_offset, fy_scale}; 

RoboClaw roboclaw(&Serial1, 1000);

scoop scoop(roboclaw, address, 
            &controller, 
            xLimPin, yLimPin, 
            x_motor_dir, y_motor_dir, 
            &pitch_motor, &vibe_motor,    
            &fx_cell, &fy_cell,          
            xMax, xMin, 
            yMax, yMin, 
            pitchMax, pitchMin, 
            vibeMin, vibeMax);

void setup() {
  Serial.begin(115200); // Print Statements
  Serial2.begin(115200); // Incoming python commands


  packet.setCallback(scoop::process_command);

  while (!Serial);
   for (int i = 0; i < 50; i++) {
    Serial.println();
  }
  scoop.init();
  pitch_motor.init(); 
  vibe_motor.init(); 
}

float accel = 10000; 
float speed = 0;
bool status = false; 


void loop() {

scoop.move_task(); 
packet.read_state_task(); 
scoop.PID_task(); 



// controller.multichannel_read();
// int duty = controller.a2.output; 
// scoop.pitchMotor->set_duty_cycle(duty);
// scoop.pitchMotor->read_angle(); 

// scoop.pitchMotor->set_angle(pitch);
// scoop.vibeMotor->set_duty_cycle(vibe); 

 
// roboclaw.SpeedAccelDeccelPositionM1M2(address, accel, speed, accel, x, accel, speed, accel, y, 0);
// scoop.vibeMotor->set_duty_cycle(vibe); 
// //Set POSITIONS
// roboclaw.SpeedAccelDeccelPositionM1M2(address, accel, speed, accel, x, accel, speed, accel, y, 0);
// scoop.pitchMotor->set_angle(pitch);

 }







//TESTING
// controller.multichannel_read();
// uint32_t y = controller.a0.output; 
// uint32_t x = controller.a1.output; 
// float vibe = controller.a2.output; 
// float pitch = controller.a3.output; 

// waypoint_t waypoint= {
//   controller.a0.output,
//   controller.a1.output,
//   controller.a2.output,
//   controller.a3.output
// };

// scoop.update_position(&waypoint); 

//Set POSITIONS
// scoop.vibeMotor->setDutyCycle(vibe); 
// scoop.update_position(waypoint_t *waypoint)
// roboclaw.SpeedAccelDeccelPositionM1M2(address, accel, speed, accel, x, accel, speed, accel, y, 0);
// scoop.pitchMotor->set_angle(pitch);

















