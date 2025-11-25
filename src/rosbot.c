/*
 * File Name: rosbot.c (Modular Version)
 * Version: 2.0.0
 * Last Modified: 25.11.2025
 * Description: Main controller integrating all modular components
 */

#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/lidar.h>
#include <webots/motor.h>
#include <webots/display.h>
#include <webots/distance_sensor.h>
#include <stdio.h>
#include <stdbool.h>

// Include all module headers
#include "config.h"
#include "astar.h"
#include "grid.h"
#include "costmap.h"
#include "frontier.h"
#include "robot.h"
#include "pid.h"
#include "display.h"

// ============== Global Variables ==============

// Grid and mapping
unsigned int grid[GRID_SIZE][GRID_SIZE];
unsigned int obstacle_counter[GRID_SIZE][GRID_SIZE];
unsigned int free_counter[GRID_SIZE][GRID_SIZE];
double costmap[GRID_SIZE][GRID_SIZE];
unsigned char frontier_edge_map[GRID_SIZE][GRID_SIZE];

// Webots devices
WbDeviceTag display;
WbDeviceTag lidar;
WbDeviceTag motors[4];
WbDeviceTag ir_sensors[4];
WbNodeRef robot_node;

// Path following variables
GridPath *current_path = NULL;
int current_waypoint = 0;
int path_update_counter = 0;
bool ir_safety_triggered = false;

// ============== Main Function ==============

int main(int argc, char *argv[]) {
    // Initialize Webots
    wb_robot_init();
    
    // Get supervisor node
    robot_node = wb_supervisor_node_get_self();
    
    // Get display device
    display = wb_robot_get_device("display");
    if (!display) {
        printf("Error: No display device found!\n");
        return 1;
    }
    
    // Get LIDAR device
    lidar = wb_robot_get_device("laser");
    if (!lidar) {
        printf("Error: No lidar device found!\n");
        return 1;
    }
    wb_lidar_enable(lidar, TIMESTEP);
    
    // Initialize all modules
    init_motors();
    init_ir_sensors();
    init_grid();
    
    int frame_count = 0;
    
    // Initial rotation (optional - can be removed if not needed)
    // rotate_drive(2, 0.3, 0.287, 0.0825);
    
    // ============== MAIN LOOP ==============
    while (wb_robot_step(TIMESTEP) != -1) {
        frame_count++;
        
        // Process LIDAR data every frame
        process_lidar();
        
        // Decay counters periodically
        if (frame_count % 10 == 0) {
            decay_counters();
            filter_connected_components(MIN_BLOB_SIZE);
        }
        
        // Update cost map periodically
        if (frame_count % 100 == 0) {
            clear_nearby_obstacles(NEARBY_RADIUS_INNER, NEARBY_RADIUS_OUTER);
            generate_costmap();
        }
        
        // Path following
        if (current_path) {
            bool path_complete = follow_path();
            
            if (path_complete) {
                printf("Goal reached! Searching for new frontier...\n");
                destroy_grid_path(current_path);
                current_path = NULL;
            }
        }
        
        // Force replan periodically if stuck or no path
        if (frame_count % 100 == 0) {
            if (current_path) {
                destroy_grid_path(current_path);
                current_path = NULL;
            }
        }
        
        // Update path to frontier if needed
        if (!current_path && frame_count % 50 == 0) {
            path_update_counter++;
            update_path_to_frontier();
        }
        
        // Render display
        if (frame_count % 5 == 0) {
            render_display_with_path(frame_count);
        }
    }
    
    // Cleanup
    if (current_path) {
        destroy_grid_path(current_path);
    }
    
    wb_robot_cleanup();
    return 0;
}
