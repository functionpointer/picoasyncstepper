#pragma once
#include<pico/stdlib.h>

#define PIOASYNCSTEPPER_BUFFER_SIZE 256
class PicoAsyncStepper {
  public:
    void begin(int pin_step, int pin_dir);
    void setEnablePin(int enablePin = 0xff);
    /// @brief Invert the function of some pins.
    void setPinsInverted(bool directionInvert = false, bool stepInvert = false, bool enableInvert = false);
    bool isDirectionInverted();

    void stop();
    void enableOutputs();
    void disableOutputs();
    void setPosition(long absolute);
    long currentPosition();  

    void moveTo(long absolute);

    void setMaxSpeed(float speed);
    float maxSpeed();

    void setAcceleration(float acceleration);
    float acceleration();

    float speed();
    float absSpeed();
    void setSpeed(float speed);

    void run();
  private:
    int pin_step=-1, pin_dir=-1, pin_en=-1;
    bool reverse_dir = false;
    bool reverse_step = false;
    bool reverse_en = false;
    bool pwm_is_running = false;
    bool running_reversed = false;
    float accel=1000;
    float max_speed;
    long current_position;
    double current_divisor=65535;

    enum OP_MODE {
      OP_MODE_STOPPING, OP_MODE_INVERTING, OP_MODE_ACCELERATING
    };

    enum TGT_TYPE {
      TGT_TYPE_POSITION, TGT_TYPE_SPEED
    };
    TGT_TYPE tgt_type=TGT_TYPE_SPEED;
    long tgt_pos=0;
    float tgt_speed=0;

    int dma_chans_top[2] = {-1, -1};
    int dma_chans_dir[2] = {-1, -1};
    uint slice_num;

    uint32_t dma_buffer_top[2][PIOASYNCSTEPPER_BUFFER_SIZE] = {{0}};
    uint32_t dma_buffer_dir[2][PIOASYNCSTEPPER_BUFFER_SIZE] = {{0}};
    int dma_running_channel=-1;
    uint32_t dir_on_value=0, dir_off_value=0;

    void setup_dma();
    void generate_tops(long distance);
    void fill_buffer_speed();
    void fill_buffer_posn();
    bool correct_direction();
    void stop_pwm();
    void start_pwm();
};
