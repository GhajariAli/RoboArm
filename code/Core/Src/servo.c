#include "servo.h"
#include <string.h>
#include <arm_math.h>

// Initialize the servo with default parameters
void init_servo(TypeServo *servo, const char *name, uint16_t offset, uint16_t span, float max_speed, float max_acceleration, float max_jerk, float servo_range) {
    strncpy(servo->name, name, sizeof(servo->name) - 1); // Copy the name
    servo->name[sizeof(servo->name) - 1] = '\0';         // Ensure null termination
    servo->current_position_degrees = 0.0f;             // Start at 0 degrees
    servo->current_speed = 0.0f;                        // No initial movement
    servo->current_acceleration = 0.0f;                 // No initial acceleration
    servo->offset = offset;                             // Set minimum PWM
    servo->span = span;                                 // Set PWM range
    servo->max_speed = max_speed;                       // Set maximum speed
    servo->max_acceleration = max_acceleration;         // Set maximum acceleration
    servo->max_jerk = max_jerk;                         // Set maximum jerk
    servo->required_accel_jerk = 0.0f;                  // Initialize required jerk for accel
    servo->required_decel_jerk = 0.0f;                  // Initialize required jerk for decel
    servo->servo_range = servo_range;                   // Set servo range of motion
    servo->target_position_degrees = 0.0f;              // Default target position
    servo->target_speed = max_speed;                    // Default target speed
    servo->output_pwm = offset;                         // Default PWM signal
    servo->profile_phase = 0;                           // Start at acceleration phase
    servo->time_elapsed = 0.0f;                         // Initialize elapsed time
    servo->decel_distance = 0.0f;                       // Initialize deceleration distance
}

// Set motion profile parameters
void set_motion_profile(TypeServo *servo, float accel_duration, float decel_duration, float target_speed) {
    servo->profile_phase = 0; // Start with acceleration phase
    servo->target_speed = target_speed; // Set user-defined cruising speed
    servo->time_elapsed = 0.0f; // Reset elapsed time

    // Calculate acceleration distance and required jerk
	servo->required_accel_jerk = 2.0f * target_speed / (accel_duration * accel_duration); //required jerk
	servo->accel_distance = servo->required_accel_jerk * accel_duration * accel_duration * accel_duration / 6.0f;
		// Calculate acceleration distance and required jerk
	servo->required_decel_jerk = 2.0f * target_speed / (decel_duration * decel_duration); //required jerk
	servo->decel_distance = servo->required_decel_jerk * decel_duration * decel_duration * decel_duration / 6.0f;

}

// Update the servo state toward the target position following a motion profile
bool update_servo_state(TypeServo *servo, float timer_interval, float deadband) {
    // Calculate position difference
    float position_diff = servo->target_position_degrees - ((servo->output_pwm - servo->offset) * servo->servo_range / servo->span);

    // If within deadband, stop
    if (fabs(position_diff) < deadband) {
        servo->current_position_degrees = servo->target_position_degrees;
        servo->current_speed = 0.0f;
        servo->current_acceleration = 0.0f;
        servo->profile_phase = 3; // Mark as complete
        return true; // Movement complete
    }

    // Increment time elapsed
    servo->time_elapsed += timer_interval;

    // Handle motion phases
    switch (servo->profile_phase) {
        case 0: // Acceleration Phase
            servo->current_acceleration += servo->required_accel_jerk * timer_interval;
            servo->current_speed += servo->current_acceleration * timer_interval;
            if (servo->current_speed >= servo->target_speed ) {
                servo->profile_phase = 1; // Move to Cruise Phase
                servo->time_elapsed = 0.0f;
            }
            break;

        case 1: // Cruise Phase
            servo->current_acceleration = 0.0f;
            servo->current_speed = servo->target_speed; // Use target speed
            if (fabs(position_diff) <= servo->decel_distance) {
                servo->profile_phase = 2; // Move to Deceleration Phase
                servo->time_elapsed = 0.0f;
                arm_sqrt_f32((6.0f * servo->required_decel_jerk * servo->decel_distance ), &servo->current_acceleration);
            }
            break;

        case 2: // Deceleration Phase
            servo->current_acceleration -= servo->required_decel_jerk * timer_interval;
            servo->current_speed -= servo->current_acceleration * timer_interval;
            if (servo->current_speed <= 0.0f) {
                servo->current_acceleration = 0.0f;
                servo->current_speed = 0.0f;
                servo->profile_phase = 3; // Mark as complete
            }
            break;

        default:
            return true; // Motion is complete
    }

    // Update speed and position
    if (fabs(servo->current_speed) > servo->target_speed) servo->current_speed = servo->target_speed * ((servo->current_speed > 0) ? 1 : -1);
    servo->current_position_degrees += servo->current_speed * timer_interval;
    servo->output_pwm = servo->offset + (uint16_t)((servo->current_position_degrees / servo->servo_range) * servo->span);

    return false; // Movement in progress
}

