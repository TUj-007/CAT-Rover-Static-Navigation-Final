#pragma once
// Verify wiring and calibrate first. No RC input is used.
constexpr bool HARDWARE_CONFIRMED=false;
constexpr bool CALIBRATION_CONFIRMED=false;
constexpr bool ESTOP_REQUIRED=true; // D30 NC contact to GND; open circuit stops.
constexpr uint8_t ESTOP_PIN=30;
// PHYSICAL order: FL, RL, FR, RR. Front=MD10; Rear=BTS7960.
constexpr uint8_t ENC_A[4]={18,19,20,21};
constexpr uint8_t ENC_B[4]={22,23,24,25};
constexpr int8_t ENC_SIGN[4]={1,1,1,1};
constexpr int8_t MOTOR_SIGN[4]={1,1,1,1};
constexpr uint8_t FL_PWM=3,FL_DIR=4,FR_PWM=9,FR_DIR=12;
constexpr uint8_t RL_RPWM=5,RL_LPWM=6,RR_RPWM=7,RR_LPWM=8;
constexpr uint8_t RL_REN=26,RL_LEN=27,RR_REN=28,RR_LEN=29;
// A-channel RISING only = x1, measured at wheel output, NOT x2/x4.
constexpr float WHEEL_DIAMETER_M[4]={.10f,.10f,.10f,.10f};
constexpr float COUNTS_PER_REV[4]={600,600,600,600};
constexpr float KP[4]={120,120,120,120},KI[4]={40,40,40,40};
constexpr float KF[4]={350,350,350,350};
// Tune separately for each motor/driver. 130 is only an Antigravity calibration lead.
constexpr uint8_t MIN_PWM[4]={130,130,130,130};
constexpr uint8_t MAX_PWM=180,PWM_RISE_PER_20MS=4;
constexpr uint16_t COMMAND_TIMEOUT_MS=300,REVERSAL_PAUSE_MS=120;
constexpr float MAX_WHEEL_MPS=.30f;
