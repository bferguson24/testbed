#include "scoop.h"
#include "utility.h"
#include <math.h>
#include <Arduino.h>
#include <Wire.h> 
#include "pid.h"
#include "taskhandler.h"

// Definitions
scoop* scoop::instance = nullptr;

//Scope resolution contructor
scoop::scoop(RoboClaw &roboclaw, uint8_t address, 
          Controller *controller,
          int xLimPin, int yLimPin, 
          int x_motor_dir, int y_motor_dir, 
          motor *pitchMotor, motor *vibeMotor,
          loadcell_t *fx_cell, loadcell_t *fy_cell, 
          float xMax, float xMin, 
          float yMax, float yMin, 
          float pitchMax, float pitchMin, 
          float vibeMin, float vibeMax)

: roboclaw(roboclaw), address(address), 
  controller(controller), 
  xLimPin(xLimPin), yLimPin(yLimPin),
  motor1dir(x_motor_dir), motor2dir(y_motor_dir), 
  pitchMotor(pitchMotor), vibeMotor(vibeMotor), 
  fx_cell(fx_cell), fy_cell(fy_cell), 
  xMax(xMax), xMin(xMin), 
  yMax(yMax), yMin(yMin), 
  pitchMax(pitchMax), pitchMin(pitchMin), 
  vibeMin(vibeMin), vibeMax(vibeMax)  
  {
  instance = this; 
}  


void scoop::init(){
  delay(1000);
  // Initialize all peripherals: 
  Serial.println("Scoop Starting");
  this->home_status = STATE_NOT_HOMED;

  

  this->vibeMotor->init();
  this->pitchMotor->init(); 

  pinMode(this->xLimPin, INPUT_PULLUP);
  pinMode(this->yLimPin, INPUT_PULLUP);

  TCCR3A = (1 << WGM31);                 // Fast PWM mode 14
  TCCR3B = (1 << WGM32) | (1 << WGM33) | (1 << CS31);  // Prescaler = 8
  ICR3 = 99;  // Set PWM period (for 20 kHz)


  // Attempt to connect to RoboClaw
  roboclaw.begin(38400);
  // if (millis() - startTime > roboclawTimeout) {
  //   Serial.println("\tERROR: RoboClaw connection timeout");
  //   return;  // Exit function or handle the error as needed
  // }
  Serial.println("Scoop Active");
  
}



void scoop::home_task(){

  uint32_t speed = trans2rot(10); // [mm/s]
  uint32_t accel = trans2rot(1000); // [mm/s^2]
  this->pitchMotor->set_angle(0); 
  this->vibeMotor->set_duty_cycle(0);

  switch (home_status){

    case STATE_START_HOME: 
      Serial.println("STARTING HOME");
    
      //Turn off Motors
      this->vibeMotor->set_duty_cycle(0); 
      this->pitchMotor->set_angle(0); 

      //Begin Homing Movement; 
      if (this->motor2dir == -1){
        this->roboclaw.BackwardM2(address, homing_speed); 
      } else {
        this->roboclaw.ForwardM2(address, homing_speed);
      }

      this->vibeMotor->set_duty_cycle(40); 

      home_status = STATE_WAITING_ENC2;  
      break;

    case STATE_WAITING_ENC1:
      Serial.print("Waiting Enc 1: Status = ");
      Serial.println(digitalRead(this->xLimPin)); 

      if (digitalRead(this->xLimPin)){
        this->roboclaw.SpeedM1M2(address, 0,0); 
        //Trying to just reset encoders at zero instead of tracking an offset;
        this->roboclaw.SetEncM1(address, 0); 
        // this->M1_lim_counts = this->roboclaw.ReadEncM1(address); 
        home_status = STATE_HOME_COMPLETE; 
        break;
        // if (this->motor2dir == -1){
        //   this->roboclaw.BackwardM2(address, homing_speed); 
        // } else {
        //   this->roboclaw.ForwardM2(address, homing_speed);
        // }
      }
      break; 

    case STATE_WAITING_ENC2:
      Serial.print("Waiting Enc 2"); 
      Serial.print("yLimPin state: ");
      Serial.println(digitalRead(this->yLimPin));
      
      if (digitalRead(this->yLimPin)){
        this->roboclaw.SpeedM1M2(address, 0,0); 
        //Trying to just reset encoders at zero instead of tracking an offset;
        this->roboclaw.SetEncM2(address, 0); 
      // this->M1_lim_counts = this->roboclaw.ReadEncM1(address); 
        home_status = STATE_WAITING_ENC1; 


        if (this->motor1dir){
          this->roboclaw.BackwardM1(address, homing_speed); 
        }
        else {
          this->roboclaw.ForwardM1(address, homing_speed);
        }
        // waypoint_t home_waypoint = {0,0,0,0}; 
        // this->update_position(&home_waypoint); 

        // this->controller->reset_channels(); 
        // this->activeCommand = COMMAND_IDLE; 
        // return;
      }
      break; 

    case STATE_PAUSE_1: 
      if (millis() - home_pause_time > 3000){
        home_status = STATE_PAUSE1_COMPLETE; 
      }
      break; 

    // case STATE_PAUSE_2: 
    //   if (millis() - home_pause_time > 3000){
    //     home_status = STATE_PAUSE2_COMPLETE; 
    //   }
    //   break; 
    
    // case STATE_PAUSE1_COMPLETE:
    //   this->vibeMotor->set_angle(-25); 
    //   this->vibeMotor->set_duty_cycle(50); 
    //   home_status = STATE_PAUSE_2; 
    //   break; 

    // case STATE_PAUSE2_COMPLETE:
    //   waypoint_t home[] = {0,0,0,0}; 
    //   this->move_waypoint(home); 
    //   home_status = STATE_HOME_COMPLETE;
    //   break; 

    // case STATE_PRE_DUMP:
    //   waypoint_t pre_dump[] = {10, 0, 0, 0}; 
    //   this->move_waypoint(pre_dump); 
    //   home_pause_time = millis(); 
    //   home_status = STATE_PAUSE_1; 
    //   break; 

    case STATE_HOME_COMPLETE: 
      waypoint_t home_waypoint = {0,0,0,0}; 
      this->update_position(&home_waypoint); 

      this->controller->reset_channels(); 
      this->activeCommand = COMMAND_IDLE; 
      break; 


    case STATE_MEASURE_YGND:
      Serial.println("Waiting for ground level"); 
      break;
  }
}

void scoop::set_command(command_t command){
this->activeCommand = command; 
}

void scoop::process_command(uint8_t buffer[]){
command_t *cmd =  (command_t*) buffer; 

switch(*cmd){
  case COMMAND_IDLE:
    // Serial.print("set cmd idle"); 
    instance->set_command(*cmd); 
    break;

  case COMMAND_START: 
    // Serial.print("set cmd start"); 
    instance->set_command(*cmd); 
    break; 

  case COMMAND_STOP: 
    // Serial.print("set cmd stop"); 
    instance->set_command(*cmd); 
    break; 

  case COMMAND_HOME: 
    // Serial.print("set cmd home"); 
    instance->set_command(*cmd); 
    instance->home_status = STATE_START_HOME; 
    break; 

  case COMMAND_MOVE_MANUAL: 
    // Serial.print("set cmd move manual"); 
    instance->set_command(*cmd); 
    break; 


  case COMMAND_MOVE_WAYPOINT: 
    // Serial.print("set cmd move waypoint"); 
    instance->activeCommand = *cmd; 

    move_cmd_t *move_cmd = (move_cmd_t*) (buffer);  
    instance->current_waypoint = &move_cmd->waypoint; 
    instance->waypoint_state = RECEIVED_WAYPOINT; 
    // Serial.print("x = ");
    // Serial.print(instance->current_waypoint->x); 
    // Serial.print("y = ");
    // Serial.print(instance->current_waypoint->y); 
    // Serial.print("pitch = ");
    // Serial.print(instance->current_waypoint->pitch); 
    // Serial.print("vibe = ");
    // Serial.print(instance->current_waypoint->vibe_speed); 
    break; 
    
  default:
    Serial.print("Unknown Command: ");
    Serial.println(*cmd);
    break;

  }
  // Serial.println(); 
}



void scoop::move_waypoint(waypoint_t *waypoint, uint32_t v_max, uint32_t a_max){


  Serial.print("x = ");
  Serial.print(waypoint->x); 
  Serial.print("y = ");
  Serial.print(waypoint->y); 
  Serial.print("pitch = ");
  Serial.print(waypoint->pitch); 
  Serial.print("vibe = ");
  Serial.println(waypoint->vibe_speed); 

  float x = waypoint->x; 
  float y = waypoint->y;
  float pitch = waypoint->pitch;
  float vibe = waypoint->vibe_speed;

  x = clip(x, xMin, xMax);
  y = clip(y, yMin, yMax); 
  pitch = clip(pitch, pitchMin, pitchMax); 
  vibe = clip(vibe, vibeMin, vibeMax); 

  int M1_dir = ((x > 0) ? 1 : -1) * scoop::motor1dir;
  int M2_dir = ((y < 0) ? 1 : -1) * scoop::motor2dir; 

  uint32_t max_accel = trans2rot(a_max);
  uint32_t M1_speed = trans2rot(v_max) * M1_dir;
  uint32_t M2_speed = 0.5 * trans2rot(v_max) * M2_dir;
  uint32_t M1_position = 2 * trans2rot(fabs(x)); 
  uint32_t M2_position  = trans2rot(fabs(y)); 



  // Serial.println("WAYPOINT CALCULATION COMPLETE"); 
  this->roboclaw.SpeedAccelDeccelPositionM1M2(address, max_accel, M1_speed, max_accel, M1_position, max_accel, M2_speed, max_accel, M2_position, 0);
  this->vibeMotor->set_duty_cycle(vibe);
  this->pitchMotor->set_angle(pitch); 

  // uint32_t M2_speed = trans2rot(vmax_y) * M2_dir;
  // uint32_t M2_accel = trans2rot(a_y);



  // Serial.print("X = ");
  // Serial.print(x); 
  // Serial.print("Y = ");
  // Serial.print(y); 
  // Serial.print("Enc counts 1 = ");
  // Serial.print(M1_position);
  // Serial.print("Enc counts 2 = ");
  // Serial.print(M2_position); 

}

void scoop::waypoint_task(){
  switch(waypoint_state){

    case WAITING_WAYPOINT: 
      break; 
    case RECEIVED_WAYPOINT:
      // Serial.print("Waypoint = ");
      // Serial.print(this->current_waypoint->x); 
      // Serial.print(",");
      // Serial.print(this->current_waypoint->y); 
      // Serial.print(",");
      // Serial.print(instance->current_waypoint->pitch); 
      // Serial.print(",");
      // Serial.println(this->current_waypoint->vibe_speed); 
      this->move_waypoint(this->current_waypoint); 
      this->waypoint_state = WAITING_WAYPOINT; 
      break;

  
  }
}


void scoop::move_task(){



// if (activeCommand == COMMAND_IDLE) {
//   Serial.println("IDLE confirmed in move_task()");
// } else {
//   Serial.print("Unexpected activeCommand = ");
//   Serial.println(activeCommand);
// }

switch (this->activeCommand){

  case COMMAND_IDLE:
    Serial.println("IDLE"); 
    break; 
  case COMMAND_START:
    Serial.println("START"); 
    break;

  case COMMAND_STOP:
    this->roboclaw.SpeedM1M2(address, 0,0); 
    Serial.println("STOP"); 
    break; 

  case COMMAND_HOME:
    // Serial.println("HOME"); 
    this->home_task(); 
    break;

  case COMMAND_MOVE_WAYPOINT:
    this->waypoint_task(); 
 
    break; 

  case COMMAND_MOVE_MANUAL:

    if (this->home_status == STATE_NOT_HOMED){
      this->activeCommand = COMMAND_HOME; 
      this->home_status = STATE_START_HOME; 
      return;
    } else {
      Serial.print("MOVE MANUAL"); 
      this->controller->multichannel_read();

      waypoint_t controller_waypoint[] = {
        this->controller->a0.output, 
        this->controller->a1.output, 
        this->controller->a2.output, 
        this->controller->a3.output
      };

      this->move_waypoint(controller_waypoint); 
      break; 
  }

}


  // switch(this->activeCommand){
  //   case COMMAND_IDLE:
  //     Serial.println("IDLE"); 
  //     this->roboclaw.SpeedM1M2(address, 0,0); 
  //     break; 
      
  //   case COMMAND_START:
  //     Serial.println("START"); 
  //     this->roboclaw.SpeedM1M2(address, 0,0); 
  //     break; 

  //   case COMMAND_STOP:
  //     Serial.println("STOP"); 
  //     this->roboclaw.SpeedM1M2(address, 0,0); 
  //     break; 

  //   case COMMAND_HOME:
  //     Serial.println("HOME"); 
  //     uint32_t speed = trans2rot(20); // [mm/s]
  //     uint32_t accel = trans2rot(1000); // [mm/s^2]

  //     switch (home_status){

  //       case STATE_START_HOME: 
  //         Serial.println("STARTING HOME");
        
  //         //Turn off Motors
  //         this->vibeMotor->set_duty_cycle(0); 
  //         this->pitchMotor->set_angle(0); 

  //         //Begin Homing Movement; 
  //         this->motor1dir == 1 ? this->roboclaw.BackwardM1(address, homing_speed) : this->roboclaw.ForwardM1(address, homing_speed);

  //         home_status = STATE_WAITING_ENC1;  
  //         break;

  //       case STATE_WAITING_ENC1:
  //         if (digitalRead(this->xLimPin)){
  //           this->roboclaw.SpeedM1M2(address, 0,0); 
  //           //Trying to just reset encoders at zero instead of tracking an offset;
  //           this->roboclaw.SetEncM1(address, 0); 
  //           // this->M1_lim_counts = this->roboclaw.ReadEncM1(address); 
  //           home_status = STATE_WAITING_ENC2; 
  //           this->motor1dir == 1 ? this->roboclaw.BackwardM1(address, homing_speed) : this->roboclaw.ForwardM1(address, homing_speed);
  //         }
  //         break; 

  //       case STATE_WAITING_ENC2:
  //         if (digitalRead(this->yLimPin)){
  //           this->roboclaw.SpeedM1M2(address, 0,0); 
  //           //Trying to just reset encoders at zero instead of tracking an offset;
  //           this->roboclaw.SetEncM2(address, 0); 
  //           // this->M1_lim_counts = this->roboclaw.ReadEncM1(address); 
  //         }
  //         home_status = STATE_MEASURE_YGND; 
  //         break; 
      
  //     }
  //     break;   

  //   case 4:
  //     Serial.println("MOVE WAYPOINT"); 
  //     // this->move_waypoint(this->current_waypoint); 
  //     break;

  //   case 5:
  //   Serial.println("MOVE MANUAL"); 
  //   this->controller->multichannel_read();

  //   waypoint_t waypoint_manual[] = {
  //     controller->a0.output,
  //     controller->a1.output,
  //     controller->a2.output,
  //     controller->a3.output
  //   };
  
  //   // this->update_position(waypoint_manual);
  //   // this->move_waypoint(waypoint_manual); 
  //   break;


  // }
  // Serial.println(); 
}





void scoop::PID_task(){
//PITCH PID 
  this->pitchMotor->read_angle();
  float output = this->pitchMotor->pid->PID_task();
  this->pitchMotor->set_duty_cycle(output); 

//FUTURE VIBE MOTOR PID
}


//Scoop Functions

void scoop::displayspeed(void){
  uint8_t status1,status2,status3,status4;
  bool valid1,valid2,valid3,valid4; 
  int32_t enc1 = scoop::roboclaw.ReadEncM1(address, &status1, &valid1);
  int32_t speed1 = scoop::roboclaw.ReadSpeedM1(address, &status2, &valid2);

  int32_t enc2 = scoop::roboclaw.ReadEncM2(address, &status3, &valid3);
  int32_t speed2 = scoop::roboclaw.ReadSpeedM2(address, &status4, &valid4);
  
  if(valid1){
    Serial.print("Encoder1:");
    Serial.print(enc1,DEC);
    Serial.print(" ");
    Serial.print(status1,HEX);
    Serial.print(" ");
  }
  if(valid2){
    Serial.print("Speed1:");
    Serial.print(speed1,DEC);
    Serial.print(" ");
  }

  if(valid3){
    Serial.print("Encoder2:");
    Serial.print(enc2,DEC);
    Serial.print(" ");
    Serial.print(status3,HEX);
    Serial.print(" ");
  }

  if(valid4){
    Serial.print("Speed2:");
    Serial.print(speed2,DEC);
    Serial.print(" ");
  }
  
  Serial.println();
}

void scoop::read_force(){
  
}


void scoop::update_position(waypoint_t *waypoint){

  this->current_waypoint = waypoint; 
// Serial.print("X: ");
// Serial.print(this->current_waypoint->x, 6); 
// Serial.print("  ");

// Serial.print("Y: ");
// Serial.print(this->current_waypoint->y, 6);
// Serial.print("  ");

// Serial.print("Pitch: ");
// Serial.print(this->current_waypoint->pitch, 6);
// Serial.print("  ");

// Serial.print("Vibe Speed: ");
// Serial.println(this->current_waypoint->vibe_speed, 6);

}

void scoop::rampVibeSpeed(){
  float timeOffset = 0; 
  int command = 0;
  float loopTime = 0;

  for (int i = 0; i <= 6; i++) {
  // Set the duty cycle for the motor (increasing by 15 each iteration)
  timeOffset = millis(); 
  loopTime = 0;

  while(loopTime < 2000){
    int duty = command + (i * 10);
    this->vibeMotor->set_duty_cycle(duty);
    loopTime = millis() - timeOffset; 
    Serial.print(millis()/1000.0f, 4);
    Serial.print(",");
    readAccel();
    Serial.println();
    }
  }
}

uint32_t scoop::trans2rot(float value){
  uint32_t output = static_cast<uint32_t>((value / this->gearLead) * this->encoder_output_ratio);
  return output;
}
uint32_t scoop::rot2trans(float value){
  uint32_t output = static_cast<uint32_t>((value * this->gearLead) / this->encoder_output_ratio);
  return output;
}



