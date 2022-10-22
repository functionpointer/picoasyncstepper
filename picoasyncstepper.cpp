#include "picoasyncstepper.h"
#include<hardware/pwm.h>
#include<pico/stdlib.h>
#include<hardware/dma.h>
#include <hardware/clocks.h>
#include<math.h>
#include<Arduino.h>

#define DIVIDE_CLOCK_BY 20
//min step freq is FREQUENCY/SLOWEST_DIVIDER, max step freq is FREQUENCY/FASTEST_DIVIDER
//examples:
//DIVIDE_CLOCK_BY 1 => min 953hz, max 19khz
//DIVIDE_CLOCK_BY 20 => min 47hz, max 964hz

#define GPIO_LEVEL 3200
#define FASTEST_DIVIDER (GPIO_LEVEL+2)
#define SLOWEST_DIVIDER 65534

#define OFF_DIVIDER (GPIO_LEVEL-1)

#define FREQUENCY ((clock_get_hz(clk_sys)/DIVIDE_CLOCK_BY)/2)
#define DIR_ON_GPIO_LEVEL 65535
#define DIR_OFF_GPIO_LEVEL 0

void PicoAsyncStepper::begin(int pin_step, int pin_dir) {
  this->pin_dir = pin_dir;
  this->pin_step = pin_step;
  gpio_init(pin_dir);
  gpio_init(pin_step);
  gpio_set_dir(pin_dir, true);
  gpio_set_dir(pin_step, true);
  running_reversed = false;

  disableOutputs();

  setMaxSpeed(-1);

  gpio_set_function(pin_step, GPIO_FUNC_PWM);
  gpio_set_function(pin_dir, GPIO_FUNC_PWM);
  this->slice_num = pwm_gpio_to_slice_num(pin_step);
  if(pwm_gpio_to_slice_num(pin_dir)!=this->slice_num) {
    while(1){
      Serial.println("step and dir must be on same pwm slice!");
      delay(100);
    }
  }
  uint dir_shift = pwm_gpio_to_channel(this->pin_dir)==0?0:16;
  uint step_shift = pwm_gpio_to_channel(this->pin_step)==0?0:16;
  this->dir_on_value = GPIO_LEVEL<<step_shift|DIR_ON_GPIO_LEVEL<<dir_shift;
  this->dir_off_value = GPIO_LEVEL<<step_shift|DIR_OFF_GPIO_LEVEL<<dir_shift;

  pwm_config c = pwm_get_default_config();
  pwm_config_set_wrap(&c, SLOWEST_DIVIDER);
  pwm_config_set_clkdiv(&c, DIVIDE_CLOCK_BY);
  pwm_config_set_clkdiv_mode(&c, PWM_DIV_FREE_RUNNING);
  pwm_config_set_phase_correct(&c, true);
  pwm_init(slice_num, &c, false);
  pwm_set_gpio_level(pin_step, GPIO_LEVEL);
  pwm_set_gpio_level(pin_dir, DIR_OFF_GPIO_LEVEL);


  for(int i=0;i<2;i++) {
    for(int j=0;j<PIOASYNCSTEPPER_BUFFER_SIZE;j++) {
      dma_buffer_top[i][j]=SLOWEST_DIVIDER;
      dma_buffer_dir[i][j]=this->dir_off_value;
    }
  }
  setup_dma();
  this->setPinsInverted(this->isDirectionInverted(), false, false);
}

void PicoAsyncStepper::setup_dma() {
  for(int i=0;i<2;i++) {
    if(dma_chans_top[i]==-1) {
      dma_chans_top[i] = dma_claim_unused_channel(true);
    }
  }
  for(int i=0;i<2;i++) {
    if(dma_chans_dir[i]==-1) {
      dma_chans_dir[i] = dma_claim_unused_channel(true);
    }
  }
     
  for(int i=0;i<2;i++) {
    dma_channel_config c = dma_channel_get_default_config(dma_chans_top[i]);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_irq_quiet(&c, false);
    channel_config_set_chain_to(&c, dma_chans_top[(i+1)%2]);
    channel_config_set_dreq(&c, pwm_get_dreq(slice_num));
    dma_channel_configure(dma_chans_top[i], &c, &pwm_hw->slice[slice_num].top, &dma_buffer_top[i], PIOASYNCSTEPPER_BUFFER_SIZE, false);
  }
  for(int i=0;i<2;i++) {
    dma_channel_config d = dma_channel_get_default_config(dma_chans_dir[i]);
    channel_config_set_transfer_data_size(&d, DMA_SIZE_32);
    channel_config_set_read_increment(&d, true);
    channel_config_set_write_increment(&d, false);
    channel_config_set_irq_quiet(&d, false);
    channel_config_set_chain_to(&d, dma_chans_dir[(i+1)%2]);
    channel_config_set_dreq(&d, pwm_get_dreq(slice_num));
    dma_channel_configure(dma_chans_dir[i], &d, &pwm_hw->slice[slice_num].cc, &dma_buffer_dir[i], PIOASYNCSTEPPER_BUFFER_SIZE, false);
  }
  dma_channel_start(dma_chans_top[0]);
  dma_channel_start(dma_chans_dir[0]);
  dma_running_channel = 0;
}

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
      dma_buffer_top[dma_running_channel][i]=dma_buffer_top[dma_running_channel][i-1];
      current_position+=running_reversed?-1:1;
      continue;
    }
    if(steps_to_go==0 && current_divisor>=65534) {
      dma_buffer_top[dma_running_channel][i]=OFF_DIVIDER;
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
      dma_buffer_top[dma_running_channel][i]=round(current_divisor);
    }
    steps_to_go = abs(tgt_pos - current_position);
  }
}

void PicoAsyncStepper::fill_buffer_speed() {
    for(int i=0;i<PIOASYNCSTEPPER_BUFFER_SIZE;i++) {
      OP_MODE opmode = OP_MODE_STOPPING;
      if(tgt_speed!=0) {
        if(this->correct_direction()) {
          double diff = abs(tgt_speed) - this->absSpeed();          
          if(diff>0) {
            opmode = OP_MODE_ACCELERATING;
          }
        } else {
          opmode = OP_MODE_INVERTING;
        }
      }
      double next = -1;
      if(opmode==OP_MODE_ACCELERATING) {
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
      dma_buffer_top[dma_running_channel][i]=round(current_divisor);
      if(dma_buffer_top[dma_running_channel][i]>SLOWEST_DIVIDER) {
        if(opmode == OP_MODE_STOPPING) {
          dma_buffer_top[dma_running_channel][i]=OFF_DIVIDER;
        } else {
          dma_buffer_top[dma_running_channel][i]=SLOWEST_DIVIDER;
          running_reversed^=true;
        }
      }
      dma_buffer_dir[dma_running_channel][i]=running_reversed?this->dir_on_value:this->dir_off_value;
      if(current_divisor>=SLOWEST_DIVIDER) {
        current_divisor=SLOWEST_DIVIDER;
      }
    }
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
        }
        fill_buffer_speed();
        dma_running_channel=(dma_running_channel+1)%2;
        fill_buffer_speed();
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
  } else if(!dma_channel_is_busy(dma_chans_top[dma_running_channel])) {
    if(tgt_type == TGT_TYPE_SPEED) {
      fill_buffer_speed();
      
    } else {//TGT_TYPE_POSITION
      fill_buffer_posn();
    }
    dma_channel_set_read_addr(dma_chans_top[dma_running_channel], &dma_buffer_top[dma_running_channel], false);
    dma_channel_set_read_addr(dma_chans_dir[dma_running_channel], &dma_buffer_dir[dma_running_channel], false);
    dma_running_channel=(dma_running_channel+1)%2;
  }
}

void PicoAsyncStepper::setPinsInverted(bool directionInvert, bool stepInvert, bool enableInvert) {
  this->reverse_dir = directionInvert;
  this->reverse_step = stepInvert;
  if(this->dma_chans_top[0]!=-1) {
    //begin() has already been called, so we are running already
    //tell hw to invert stuff so we don't actually have to deal with it lol
    if(this->reverse_en!=enableInvert) {
      this->reverse_en = enableInvert;
      disableOutputs();
    }
    
    bool invert[2] = {false};
    invert[pwm_gpio_to_channel(this->pin_dir)]=reverse_dir;
    invert[pwm_gpio_to_channel(this->pin_step)]=reverse_step;
    pwm_set_output_polarity(this->slice_num, invert[0], invert[1]);
  }
  this->reverse_en = enableInvert;
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
    gpio_put(pin_en^reverse_en, false);
  }
}

void PicoAsyncStepper::disableOutputs() {
  if(pin_en>0) {
    gpio_put(pin_en^reverse_en, true);
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
  float max = FREQUENCY/FASTEST_DIVIDER;//limit for how fast we can go
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
