#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include <stdint.h>
#include <stdbool.h>

// Struct to define a servo motor and its parameters
typedef struct TypeServo {
    char name[32];                 // Name or identifier for the servo
    float current_position_degrees;  // Current position in degrees
    float current_speed;             // Current speed in degrees per second
    float current_acceleration;      // Current acceleration in degrees per second^2
    float max_speed;                 // Maximum speed in degrees per second
    float max_acceleration;          // Maximum acceleration in degrees per second^2
    float required_accel_jerk;       // Required jerk in degrees per second^3 in acceleration
    float required_decel_jerk;       // Required jerk in degrees per second^3 in deceleration
    float max_jerk;                  // Maximum jerk in degrees per second^3
    uint16_t offset;                 // Minimum PWM value
    uint16_t span;                   // Range of PWM values (offset + span = max PWM)
    float servo_range;               // Servo range of motion in degrees (e.g., 180 degrees)
    float target_position_degrees;   // Desired position in degrees
    float target_speed;              // Desired speed in degrees per second
    uint16_t output_pwm;             // Current PWM output signal (e.g., for the servo driver)
    uint8_t profile_phase;           // 0: Accel, 1: Cruise, 2: Decel
    float time_elapsed;              // Time elapsed in the current phase
    float accel_distance; 			 // Precomputed acceleration distance
    float decel_distance;            // Precomputed deceleration distance
} TypeServo;

void init_servo(TypeServo *servo, const char *name, uint16_t offset, uint16_t span, float max_speed, float max_acceleration, float max_jerk, float servo_range);
void set_motion_profile(TypeServo *servo, float accel_duration, float decel_duration, float target_speed);
bool update_servo_state(TypeServo *servo, float timer_interval, float deadband);

#endif
