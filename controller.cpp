#include "pins_arduino.h"
#include "controller.h"
#include "utility.h" 


Controller::Controller(float deadband, float stepsize)
  : deadzone(deadzone), stepsize(stepsize)
{
  a0 = {0, A0, (int)deadband, 0.25, 1, 700, -1};
  a1 = {0, A1, (int)deadband, 0.25, 1, 190, 1};
  a2 = {0, A2, (int)deadband, 0.1, -30, 30, -1};
  a3 = {0, A3, (int)deadband, 0.3, 0, 100, 1};

  channels[0] = &a0;
  channels[1] = &a1;
  channels[2] = &a2;
  channels[3] = &a3;

};

void Controller::analog_step(adc_channel_t *channel){
  int minVal = 0.0f;
  int maxVal = 1023.0f;
  int rawVal = analogRead(channel->analog_pin);
  float avgVal = (minVal + maxVal) / 2; 

  float dval = 0.0; 

  if (rawVal > avgVal + channel->deadband){
    dval = (rawVal - (avgVal + channel->deadband)) / (1023.0 - (avgVal + channel->deadband)) * channel->stepsize;
  }
  else if (rawVal < (avgVal - channel->deadband)){
    dval = (rawVal - avgVal) / (avgVal - 0.0) * channel->stepsize;
  }
  
  channel->output += channel->invert_channel * dval; 
  channel->output = clip(channel->output, channel->min, channel->max); 
}


void Controller::reset_channels(){
  for (int i = 0; i < n_channels; i++){
    this->channels[i]->output = 0; 
  }
}


void Controller::multichannel_read(){
  analog_step(&this->a0); 
  analog_step(&this->a1); 
  analog_step(&this->a2); 
  analog_step(&this->a3); 

  // Serial.print(this->a0.output);
  // Serial.print(","); 
  // Serial.print(this->a1.output);
  // Serial.print(","); 
  // Serial.print(this->a2.output);
  // Serial.print(","); 
  // Serial.println(this->a3.output);
  
}


