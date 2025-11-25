#include "robot.h"
#include "pid.h"
#include "grid.h"
#include "config.h"
#include "astar.h"
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/supervisor.h>
#include <webots/robot.h>
#include <math.h>
#include <stdio.h>

// External globals
extern WbDeviceTag motors[4];
extern WbDeviceTag ir_sensors[4];
extern WbNodeRef robot_node;
extern GridPath *current_path;
extern int current_waypoint;
extern bool ir_safety_triggered;

static const char *ir_sensor_names[4] = {"fl_range", "fr_range", "rl_range", "rr_range"};

void init_motors(void) {
    motors[0] = wb_robot_get_device("fl_wheel_joint");
    motors[1] = wb_robot_get_device("rl_wheel_joint");
    motors[2] = wb_robot_get_device("fr_wheel_joint");
    motors[3] = wb_robot_get_device("rr_wheel_joint");
    
    for (int i = 0; i < 4; i++) {
        wb_motor_set_position(motors[i], INFINITY);
        wb_motor_set_velocity(motors[i], 0.0);
    }
}

void init_ir_sensors(void) {
    for (int i = 0; i < 4; i++) {
        ir_sensors[i] = wb_robot_get_device(ir_sensor_names[i]);
        if (ir_sensors[i]) {
            wb_distance_sensor_enable(ir_sensors[i], TIMESTEP);
            printf("IR sensor %s initialized\n", ir_sensor_names[i]);
        } else {
            printf("Warning: IR sensor %s not found!\n", ir_sensor_names[i]);
        }
    }
}

bool check_ir_safety(double *min_distance, int *sensor_index) {
    *min_distance = INFINITY;
    *sensor_index = -1;
    bool safe = true;
    
    for (int i = 0; i < 4; i++) {
        if (!ir_sensors[i]) continue;
        
        double distance = wb_distance_sensor_get_value(ir_sensors[i]);
        
        if (distance < *min_distance) {
            *min_distance = distance;
            *sensor_index = i;
        }
        
        if (distance < IR_SAFETY_THRESHOLD) {
            safe = false;
        }
    }
    
    return safe;
}

void emergency_stop(void) {
    for (int i = 0; i < 4; i++) {
        wb_motor_set_velocity(motors[i], 0.0);
    }
    printf("EMERGENCY STOP: IR safety triggered!\n");
}

double clamp(double val, double min, double max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

double normalize_angle(double angle) {
    return atan2(sin(angle), cos(angle));
}

double rotate_drive(double rotations, double angle_threshold, double b, double r) {
    // Rotation in radians (full rotations)
    double radians_target = rotations * 2.0 * M_PI;
    
    // Current orientation based on rotation matrix
    const double *orientation = wb_supervisor_node_get_orientation(robot_node);
    double robot_theta = atan2(orientation[3], orientation[0]);
    
    // Angle difference
    double angle_diff = normalize_angle(radians_target - robot_theta);
    
    // --- stop condition ---
    if (fabs(angle_diff) < angle_threshold) {
        // Stop motors
        wb_motor_set_velocity(motors[0], 0);
        wb_motor_set_velocity(motors[1], 0);
        wb_motor_set_velocity(motors[2], 0);
        wb_motor_set_velocity(motors[3], 0);
        return 1.0; // success
    }
    
    // PID angular velocity command
    double omega = pid_rotate(angle_diff);
    
    // Convert body angular velocity to wheel velocities
    double omega_l = -omega * (b / (2.0 * r));
    double omega_r = omega * (b / (2.0 * r));
    
    // Set wheel velocities correctly
    wb_motor_set_velocity(motors[0], omega_l);  // left front
    wb_motor_set_velocity(motors[1], omega_l);  // left rear
    wb_motor_set_velocity(motors[2], omega_r);  // right front
    wb_motor_set_velocity(motors[3], omega_r);  // right rear
    
    return 0.0; // still rotating
}

bool diff_drive(int x_goal, int y_goal, double distance_threshold, double angle_threshold, double b, double r) {
    // IR Safety Check
    double min_ir_distance;
    int triggered_sensor;
    bool ir_safe = check_ir_safety(&min_ir_distance, &triggered_sensor);
    
    if (!ir_safe) {
        if (min_ir_distance < IR_CRITICAL_THRESHOLD) {
            // Critical distance - emergency stop
            emergency_stop();
            ir_safety_triggered = true;
            printf("CRITICAL: IR sensor %s detected obstacle at %.3fm - EMERGENCY STOP\n", 
                   ir_sensor_names[triggered_sensor], min_ir_distance);
            return false;
        } else {
            // Warning distance - slow down significantly
            printf("WARNING: IR sensor %s detected obstacle at %.3fm - reducing speed\n", 
                   ir_sensor_names[triggered_sensor], min_ir_distance);
            ir_safety_triggered = true;
        }
    } else {
        ir_safety_triggered = false;
    }
    
    const double *position = wb_supervisor_node_get_position(robot_node);
    const double *orientation = wb_supervisor_node_get_orientation(robot_node);
    
    double robot_x = position[0];
    double robot_y = position[1];
    double robot_theta = atan2(orientation[3], orientation[0]);
    
    int robot_gx, robot_gy;
    world_to_grid(robot_x, robot_y, &robot_gx, &robot_gy);
    
    double dx = x_goal - robot_gx;
    double dy = y_goal - robot_gy;
    double distance = sqrt(dx * dx + dy * dy);
    
    double angle_to_goal = atan2(dy, dx);
    double angle_diff = normalize_angle(angle_to_goal - robot_theta);
    
    if (distance < distance_threshold) {
        for (int i = 0; i < 4; i++) {
            wb_motor_set_velocity(motors[i], 0.0);
        }
        return true;
    }
    
    double omega, speed;
    if (fabs(angle_diff) > angle_threshold) {
        omega = pid_rotate(angle_diff);
        speed = 0.0;
    } else {
        omega = pid_rotate(angle_diff);
        speed = pid_translate(distance);
    }
    
    // Apply IR safety speed reduction
    if (ir_safety_triggered && min_ir_distance < IR_SAFETY_THRESHOLD) {
        // Scale speed based on proximity
        double safety_factor = (min_ir_distance - IR_CRITICAL_THRESHOLD) / 
                              (IR_SAFETY_THRESHOLD - IR_CRITICAL_THRESHOLD);
        safety_factor = clamp(safety_factor, 0.0, 1.0);
        speed *= safety_factor * 0.3; // Reduce to max 30% of normal speed
        omega *= 0.5; // Also reduce rotation speed
    }
    
    double omega_l = speed - omega * (b / (2.0 * r));
    double omega_r = speed + omega * (b / (2.0 * r));
    
    wb_motor_set_velocity(motors[0], omega_l);
    wb_motor_set_velocity(motors[1], omega_l);
    wb_motor_set_velocity(motors[2], omega_r);
    wb_motor_set_velocity(motors[3], omega_r);
    
    return false;
}

bool follow_path(void) {
    if (!current_path || current_waypoint >= current_path->count) {
        return false;
    }
    
    // Track if we're making progress
    static int stuck_counter = 0;
    static int last_waypoint = -1;
    
    if (current_waypoint == last_waypoint) {
        stuck_counter++;
        if (stuck_counter > 50) { // Stuck for 50 frames
            printf("Path following stuck, replanning...\n");
            destroy_grid_path(current_path);
            current_path = NULL;
            stuck_counter = 0;
            last_waypoint = -1;
            return true; // Force replan
        }
    } else {
        stuck_counter = 0;
    }
    last_waypoint = current_waypoint;
    
    const double *position = wb_supervisor_node_get_position(robot_node);
    int robot_gx, robot_gy;
    world_to_grid(position[0], position[1], &robot_gx, &robot_gy);
    
    // Skip close waypoints
    while (current_waypoint < current_path->count - 1) {
        GridNode *next_wp = &current_path->nodes[current_waypoint];
        double dx = next_wp->x - robot_gx;
        double dy = next_wp->y - robot_gy;
        double dist = sqrt(dx * dx + dy * dy);
        
        if (dist < 10.0) {
            current_waypoint++;
            reset_pid_controllers();
        } else {
            break;
        }
    }
    
    GridNode *waypoint = &current_path->nodes[current_waypoint];
    bool reached = diff_drive(waypoint->x, waypoint->y, 10.0, 1.0, 0.287, 0.0825);
    
    if (reached) {
        current_waypoint++;
        printf("Reached waypoint %d/%d\n", current_waypoint, current_path->count);
    }
    
    if (current_waypoint >= current_path->count) {
        printf("Path completed!\n");
        return true;
    }
    
    return false;
}
