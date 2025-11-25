#ifndef DISPLAY_H
#define DISPLAY_H

#include "astar.h"
#include <webots/types.h>

// Display rendering functions
void render_display_with_path(int frame_count);
unsigned int cost_to_color(double cost);
unsigned int frontier_score_to_color(double score, double min_score, double max_score);
void draw_path_on_display(WbDeviceTag display, GridPath *path, int robot_gx, int robot_gy, 
                          double scale, int grid_start_x, int grid_start_y);
void display_ir_status(int frame_count);

#endif // DISPLAY_H
