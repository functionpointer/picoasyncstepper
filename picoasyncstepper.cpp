#include "picoasyncstepper.h"
#include<hardware/pwm.h>
#include<pico/stdlib.h>
#include<hardware/dma.h>
#include <hardware/clocks.h>
#include<math.h>
#include<Arduino.h>

#define GPIO_LEVEL 3200
#define OFF_DIVIDER (GPIO_LEVEL-1)

void PicoAsyncStepper::begin(int pin_step, int pin_dir) {
  this->pin_dir = pin_dir;
  this->pin_step = pin_step;
  gpio_init(pin_dir);
  gpio_init(pin_step);
  gpio_set_dir(pin_dir, true);
  gpio_set_dir(pin_step, true);
  gpio_put(pin_dir, false^this->reverse_dir);
  running_reversed = false;

  disableOutputs();

  setMaxSpeed(-1);

  gpio_set_function(pin_step, GPIO_FUNC_PWM);
  slice_num = pwm_gpio_to_slice_num(pin_step);

  pwm_config c = pwm_get_default_config();
  pwm_config_set_wrap(&c, 30000);
  pwm_config_set_clkdiv(&c, 1);
  pwm_config_set_clkdiv_mode(&c, PWM_DIV_FREE_RUNNING);
  pwm_config_set_phase_correct(&c, true);
  pwm_init(slice_num, &c, false);
  pwm_set_gpio_level(pin_step, GPIO_LEVEL);
  for(int i=0;i<2;i++) {
    for(int j=0;j<PIOASYNCSTEPPER_BUFFER_SIZE;j++) {
      dma_buffer[i][j]=65000;
    }
  }
  setup_dma();
}

void PicoAsyncStepper::setup_dma() {
  for(int i=0;i<2;i++) {
    if(dma_chans[i]==-1) {
      dma_chans[i] = dma_claim_unused_channel(true);
    }
  }
  for(int i=0;i<2;i++) {
    dma_channel_config c = dma_channel_get_default_config(dma_chans[i]);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_irq_quiet(&c, false);
    channel_config_set_chain_to(&c, dma_chans[(i+1)%2]);
    channel_config_set_dreq(&c, pwm_get_dreq(slice_num));
    dma_channel_configure(dma_chans[i], &c, &pwm_hw->slice[slice_num].top, &dma_buffer[i], PIOASYNCSTEPPER_BUFFER_SIZE, false);
  }
  dma_channel_start(dma_chans[0]);
  dma_running_channel = 0;
}

#define FREQUENCY (clock_get_hz(clk_sys)/2)
double next_divisor(double current_divisor) {
  return FREQUENCY*current_divisor/(FREQUENCY+current_divisor);
}

double prev_divisor(double current_divisor) {
  return FREQUENCY/(FREQUENCY/current_divisor-1);
}

int steps_to_stop(float speed, float accel) {
  float time_to_stop = speed/accel;
  float distance_to_stop = speed*time_to_stop + 0.5*accel*time_to_stop*time_to_stop;
  return distance_to_stop/2;
}

bool PicoAsyncStepper::correct_direction() {
  if(tgt_type==TGT_TYPE_POSITION) {
    return (tgt_pos-current_position>0) ^ running_reversed;
  } else {
    return tgt_speed>0 ^ running_reversed;
  }
}
#define FAST_THRESHOLD 14000
#define FAST_FACTOR 8
void PicoAsyncStepper::fill_buffer_posn() {
  long steps_to_go = abs(tgt_pos - current_position);
  for(int i=0;i<PIOASYNCSTEPPER_BUFFER_SIZE;i++) {
    if(i%FAST_FACTOR!=0 && current_divisor<FAST_THRESHOLD) {
      dma_buffer[dma_running_channel][i]=dma_buffer[dma_running_channel][i-1];
      current_position+=running_reversed?-1:1;
      continue;
    }
    if(steps_to_go==0 && current_divisor>=65534) {
      dma_buffer[dma_running_channel][i]=OFF_DIVIDER;
    } else {
      int braking_point = steps_to_stop(this->absSpeed(), this->accel);
      float next = 0;
      bool accel = false;
      if(this->correct_direction() && braking_point<steps_to_go) {
        double speed = FREQUENCY/current_divisor;
        double diff = this->max_speed - this->absSpeed();
        accel = diff>0;
      } 
      if(accel) {
        next = next_divisor(current_divisor);
      } else {
        next = prev_divisor(current_divisor);
      }
      double difference = next-current_divisor;//spread this difference out to a whole second
      double new_freq = FREQUENCY / current_divisor;
      if(current_divisor<FAST_THRESHOLD) {
        new_freq /= FAST_FACTOR;
      }
      difference /= new_freq;//by dividing by the number of times this happens per second i.e. frequency
      current_divisor+=difference*this->accel;
      if(current_divisor>=65536) {
        current_divisor=65536-1;//don't stop!
      }
      current_position+=running_reversed?-1:1;
      dma_buffer[dma_running_channel][i]=round(current_divisor);
    }
    steps_to_go = abs(tgt_pos - current_position);
  }
}

void PicoAsyncStepper::fill_buffer_speed() {
    for(int i=0;i<PIOASYNCSTEPPER_BUFFER_SIZE;i++) {
      bool accel = false;
      if(this->correct_direction()) {
        double diff = abs(tgt_speed) - this->absSpeed();
        accel = diff>0;
      }
      double next = -1;
      if(accel) {
        //accelerate
        next = next_divisor(current_divisor);
      } else {
        //decelerate
        next = prev_divisor(current_divisor);
      }
      double difference = next-current_divisor;//spread this difference out to a whole second
      double new_freq = FREQUENCY / next;
      difference /= new_freq;//by dividing by the number of times this happens per second i.e. frequency
      current_divisor+=difference*this->accel;
      if(current_divisor>=65536) {
        current_divisor=65536;
      }
      dma_buffer[dma_running_channel][i]=round(current_divisor);
      if(dma_buffer[dma_running_channel][i]>65535) {
         dma_buffer[dma_running_channel][i]=OFF_DIVIDER;
      }
    }
    pwm_set_enabled(slice_num, true);
}

void PicoAsyncStepper::stop_pwm() {
  pwm_set_enabled(slice_num, false);
  setup_dma();//reset dma for clean restart
  dma_running_channel = 0;
  pwm_is_running = false;
}

void PicoAsyncStepper::start_pwm() {
    pwm_set_enabled(slice_num, true);
    pwm_is_running = true;
    dma_running_channel = 0;
}

//https://stackoverflow.com/questions/2922619/how-to-efficiently-compare-the-sign-of-two-floating-point-values-while-handling
bool samesign(double a, double b) {
  return a*b >= 0.0;
}

void PicoAsyncStepper::run() {
  if(!pwm_is_running) {
    if(tgt_type==TGT_TYPE_SPEED) {
      if(abs(tgt_speed)>0) {
        current_divisor=65535;
        if(tgt_speed<0) {
          running_reversed = true;
          gpio_put(pin_dir, running_reversed^this->reverse_dir);
        }
        fill_buffer_speed();
        dma_running_channel=(dma_running_channel+1)%2;
        start_pwm();
      }
    } else {
      if(tgt_pos != current_position) {
        if(!this->correct_direction()) {
          running_reversed ^= true;
          gpio_put(pin_dir, running_reversed^this->reverse_dir);
        }
        current_divisor=65535;
        fill_buffer_posn();
        dma_running_channel=(dma_running_channel+1)%2;
        start_pwm();
      }
    }
  } else if(!dma_channel_is_busy(dma_chans[dma_running_channel])) {
    if(current_divisor >= 65535 && !this->correct_direction()) {
      running_reversed^=true;
      gpio_put(pin_dir, running_reversed^this->reverse_dir);
    }
    if(tgt_type == TGT_TYPE_SPEED) {
      fill_buffer_speed();
      
    } else {//TGT_TYPE_POSITION
      fill_buffer_posn();
    }
    dma_channel_set_read_addr(dma_chans[dma_running_channel], &dma_buffer[dma_running_channel], false);
    dma_running_channel=(dma_running_channel+1)%2;
  }
}

void PicoAsyncStepper::setPinsInverted(bool directionInvert, bool stepInvert, bool enableInvert) {
  this->reverse_dir = directionInvert;
  gpio_put(pin_dir, running_reversed^this->reverse_dir);
}

bool PicoAsyncStepper::isDirectionInverted() {
  return this->reverse_dir;
}

void PicoAsyncStepper::setEnablePin(int pin_en) {
  this->pin_en = pin_en;
  gpio_init(pin_en);
  gpio_set_dir(pin_en, true);
  disableOutputs();
}

void PicoAsyncStepper::stop() {
  tgt_speed = 0;
  tgt_type = TGT_TYPE_SPEED;
}

void PicoAsyncStepper::enableOutputs() {
  if(pin_en>0) {
    gpio_put(pin_en, false);
  }
}

void PicoAsyncStepper::disableOutputs() {
  if(pin_en>0) {
    gpio_put(pin_en, true);
  }
}
void PicoAsyncStepper::setPosition(long absolute) {
  current_position = absolute;
}
long PicoAsyncStepper::currentPosition() {
  return current_position;
}

void PicoAsyncStepper::moveTo(long absolute) {
  tgt_type = TGT_TYPE_POSITION;
  tgt_pos = absolute;
}

void PicoAsyncStepper::setMaxSpeed(float speed) {
  float max = FREQUENCY/GPIO_LEVEL-80;//limit for how fast we can go
  if(speed>max || speed<0) {
    speed=max;
  }
  max_speed = speed;
}
float PicoAsyncStepper::maxSpeed() {
  return max_speed;
}

void PicoAsyncStepper::setAcceleration(float acceleration) {
  accel = acceleration;
}

float PicoAsyncStepper::acceleration() {
  return accel;
}

float PicoAsyncStepper::absSpeed() {
  if(current_divisor==0) {
    return 0;
  } else {
    return FREQUENCY/current_divisor;
  }
}

float PicoAsyncStepper::speed() {
  return (running_reversed?-1:1)*absSpeed();
}

void PicoAsyncStepper::setSpeed(float speed) {
  if(speed>maxSpeed()) {
    speed=maxSpeed();
  }
  tgt_speed = speed;
  tgt_type=TGT_TYPE_SPEED;
}
