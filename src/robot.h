#ifndef ROBOT_H
#define ROBOT_H

#include <stdbool.h>
#include <webots/types.h>

// Robot control functions
void init_motors(void);
void init_ir_sensors(void);
void emergency_stop(void);
bool check_ir_safety(double *min_distance, int *sensor_index);

// Differential drive control
bool diff_drive(int x_goal, int y_goal, double distance_threshold, double angle_threshold, double b, double r);
double rotate_drive(double rotations, double angle_threshold, double b, double r);
bool follow_path(void);

// Utility functions
double clamp(double val, double min, double max);
double normalize_angle(double angle);

#endif // ROBOT_H
