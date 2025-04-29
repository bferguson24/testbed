#pragma once
#include <stdint.h>
#include <Arduino.h> 

typedef struct{
  float output; 
  int analog_pin; 
  int deadband; 
  float stepsize;
  float min;
  float max; 
  int invert_channel; 
}adc_channel_t; 



class Controller{
  public:

  static constexpr int n_channels = 4; 
  adc_channel_t a0, a1, a2, a3;
  adc_channel_t* channels[n_channels]; 

  Controller(float deadband, float stepsize); 
  
  void analog_step(adc_channel_t *channel); 
  void reset_channels(); 
  void multichannel_read();

  private: 
    float stepsize;
    float deadzone; 
};