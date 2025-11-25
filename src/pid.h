#ifndef PID_H
#define PID_H

// PID controller functions
double pid_rotate(double angle_error);
double pid_translate(double distance_error);
void reset_pid_controllers(void);

#endif // PID_H
