#include "pid.h"
#include "robot.h"
#include "config.h"
#include <math.h>

// PID state variables
static double pid_rotate_integral = 0.0;
static double pid_rotate_prev_error = 0.0;
static double pid_translate_integral = 0.0;
static double pid_translate_prev_error = 0.0;

double pid_rotate(double angle_error) {
    static double Kp_rotate = 1.0;
    static double Ki_rotate = 0.0;
    static double Kd_rotate = 0.0;
    
    double integral_limit = 1.0;
    
    // Integral term
    pid_rotate_integral += angle_error * (TIMESTEP / 1000.0);
    pid_rotate_integral = clamp(pid_rotate_integral, -integral_limit, integral_limit);
    
    // Derivative term
    double derivative = (angle_error - pid_rotate_prev_error) / (TIMESTEP / 1000.0);
    pid_rotate_prev_error = angle_error;
    
    // PID output
    double omega = Kp_rotate * angle_error + Ki_rotate * pid_rotate_integral + Kd_rotate * derivative;
    
    return omega;
}

double pid_translate(double distance_error) {
    static double Kp_translate = 0.02;
    static double Kd_translate = 0.0;
    
    // Derivative term
    double derivative = (distance_error - pid_translate_prev_error) / (TIMESTEP / 1000.0);
    pid_translate_prev_error = distance_error;
    
    // PD output (no integral for translation)
    double speed = Kp_translate * distance_error + Kd_translate * derivative;
    
    return clamp(speed, 0, 2.0);
}

void reset_pid_controllers(void) {
    pid_rotate_integral = 0.0;
    pid_rotate_prev_error = 0.0;
    pid_translate_integral = 0.0;
    pid_translate_prev_error = 0.0;
}
