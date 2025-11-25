#include "display.h"
#include "config.h"
#include "grid.h"
#include "costmap.h"
#include "frontier.h"
#include "robot.h"
#include <webots/display.h>
#include <webots/supervisor.h>
#include <webots/distance_sensor.h>
#include <math.h>
#include <stdio.h>

// External globals
extern WbDeviceTag display;
extern WbNodeRef robot_node;
extern unsigned int grid[GRID_SIZE][GRID_SIZE];
extern double costmap[GRID_SIZE][GRID_SIZE];
extern GridPath *current_path;
extern int current_waypoint;
extern WbDeviceTag ir_sensors[4];
extern bool ir_safety_triggered;

static const char *ir_sensor_names[4] = {"fl_range", "fr_range", "rl_range", "rr_range"};

unsigned int cost_to_color(double cost) {
    double min_C = FREE_COST;
    double max_C = OBSTACLE_COST;
    double norm = (cost - min_C) / (max_C - min_C);
    
    if (norm < 0.0) norm = 0.0;
    if (norm > 1.0) norm = 1.0;
    
    int r, g, b;
    
    if (norm < 0.33) {
        // White -> Yellow
        double t = norm / 0.33; // 0 -> 1
        r = 255;
        g = (int)(255 - (1-t) * 255); // constant
        b = (int)(255 * (1 - t)); // 255 -> 0
    }
    else if (norm < 0.66) {
        // Yellow -> Orange
        double t = (norm - 0.33) / 0.33; // 0 -> 1
        r = 255;
        g = (int)(255 - (255 - 165) * t); // 255 -> 165
        b = 0;
    }
    else {
        // Orange -> Red
        double t = (norm - 0.66) / 0.34; // 0 -> 1
        r = 255;
        g = (int)(165 * (1 - t)); // 165 -> 0
        b = 0;
    }
    
    // Clamp safety
    if (r < 0) r = 0;
    if (r > 255) r = 255;
    if (g < 0) g = 0;
    if (g > 255) g = 255;
    if (b < 0) b = 0;
    if (b > 255) b = 255;
    
    return ((unsigned int)r << 16) | ((unsigned int)g << 8) | (unsigned int)b;
}

unsigned int frontier_score_to_color(double score, double min_score, double max_score) {
    double norm = 0.0;
    if (max_score > min_score) {
        norm = (score - min_score) / (max_score - min_score);
    }
    if (norm < 0.0) norm = 0.0;
    if (norm > 1.0) norm = 1.0;
    
    int r, g, b;
    if (norm < 0.2) {
        double t = norm * 5.0;
        r = (int)(255.0 * t);
        g = 255;
        b = 0;
    } else {
        double t = (norm - 0.2) / 0.2;
        r = 255;
        g = (int)(255.0 * (1.0 - t));
        b = 0;
    }
    
    if (r < 0) r = 0;
    if (r > 255) r = 255;
    if (g < 0) g = 0;
    if (g > 255) g = 255;
    if (b < 0) b = 0;
    if (b > 255) b = 255;
    
    return ((unsigned int)r << 16) | ((unsigned int)g << 8) | (unsigned int)b;
}

void draw_path_on_display(WbDeviceTag display, GridPath *path, int robot_gx, int robot_gy, 
                          double scale, int grid_start_x, int grid_start_y) {
    if (!path) return;
    
    wb_display_set_color(display, 0x00FF00);
    for (int i = 0; i < path->count - 1; i++) {
        GridNode n1 = path->nodes[i];
        GridNode n2 = path->nodes[i + 1];
        
        int x1 = (int)((n1.x - grid_start_x) * scale);
        int y1 = DISPLAY_HEIGHT - (int)((n1.y - grid_start_y + 1) * scale);
        int x2 = (int)((n2.x - grid_start_x) * scale);
        int y2 = DISPLAY_HEIGHT - (int)((n2.y - grid_start_y + 1) * scale);
        
        wb_display_draw_line(display, x1, y1, x2, y2);
    }
    
    wb_display_set_color(display, 0x00FFFF);
    for (int i = 0; i < path->count; i++) {
        GridNode n = path->nodes[i];
        int x = (int)((n.x - grid_start_x) * scale);
        int y = DISPLAY_HEIGHT - (int)((n.y - grid_start_y + 1) * scale);
        
        if (i == current_waypoint) {
            wb_display_set_color(display, 0xFF00FF);
            wb_display_fill_oval(display, x - 3, y - 3, 6, 6);
        } else {
            wb_display_set_color(display, 0x0000FF);
            wb_display_draw_oval(display, x - 2, y - 2, 4, 4);
        }
    }
}

void display_ir_status(int frame_count) {
    double min_distance;
    int sensor_idx;
    check_ir_safety(&min_distance, &sensor_idx);
    
    int status_x = 6;
    int status_y = DISPLAY_HEIGHT - 60;
    
    // Background box for IR status
    if (ir_safety_triggered) {
        wb_display_set_color(display, 0xFF0000);
        wb_display_fill_rectangle(display, status_x - 2, status_y - 2, 180, 55);
    }
    
    wb_display_set_color(display, 0xFFFFFF);
    wb_display_set_font(display, "Arial", 10, 0);
    wb_display_draw_text(display, "IR Sensors", status_x, status_y);
    
    for (int i = 0; i < 4; i++) {
        if (!ir_sensors[i]) continue;
        
        double dist = wb_distance_sensor_get_value(ir_sensors[i]);
        char sensor_info[64];
        sprintf(sensor_info, "%s: %.3fm", ir_sensor_names[i], dist);
        
        // Color code based on distance
        if (dist < IR_CRITICAL_THRESHOLD) {
            wb_display_set_color(display, 0xFF0000); // Red
        } else if (dist < IR_SAFETY_THRESHOLD) {
            wb_display_set_color(display, 0xFFAA00); // Orange
        } else {
            wb_display_set_color(display, 0x00FF00); // Green
        }
        
        wb_display_draw_text(display, sensor_info, status_x, status_y + 12 + i * 10);
    }
    
    // Status message
    if (ir_safety_triggered) {
        wb_display_set_color(display, 0xFFFF00);
        wb_display_draw_text(display, "SAFETY ACTIVE", status_x + 100, status_y + 20);
    }
}

void render_display_with_path(int frame_count) {
    // Lazy one-time font initialization to avoid repeated font allocations
    static bool font_initialized = false;
    if (!font_initialized) {
        wb_display_set_font(display, "Arial", 12, 0);
        font_initialized = true;
    }
    
    // Clear background
    wb_display_set_color(display, COLOR_BACKGROUND);
    wb_display_fill_rectangle(display, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
    
    const double *position = wb_supervisor_node_get_position(robot_node);
    double robot_x = position[0];
    double robot_y = position[1];
    
    int robot_gx, robot_gy;
    world_to_grid(robot_x, robot_y, &robot_gx, &robot_gy);
    
    double scale = 3.0;
    int viewport_size = (int)(DISPLAY_WIDTH / scale);
    int half_viewport = viewport_size / 2;
    int cell_size = (int)(scale + 1);
    
    int grid_start_x = robot_gx - half_viewport;
    int grid_end_x = robot_gx + half_viewport;
    int grid_start_y = robot_gy - half_viewport;
    int grid_end_y = robot_gy + half_viewport;
    
    // Draw visible cells. Minimize setcolor calls by tracking last color.
    unsigned int last_color = 0xFFFFFFFF; // impossible to match
    
    for (int gy = grid_start_y; gy <= grid_end_y; gy++) {
        for (int gx = grid_start_x; gx <= grid_end_x; gx++) {
            if (!is_valid_cell(gx, gy)) continue;
            
            unsigned int color;
            unsigned int cell = grid[gy][gx];
            
            if (cell == CELL_UNKNOWN) {
                continue; // skip unknown for performance
            } else if (cell == CELL_OBSTACLE) {
                color = COLOR_OBSTACLE;
            } else if (cell == CELL_ROBOT) {
                continue; // robot rendered later at center
            } else { // free or others
                double c = costmap[gy][gx];
                color = cost_to_color(c);
            }
            
            if (color != last_color) {
                wb_display_set_color(display, color);
                last_color = color;
            }
            
            int screen_x = (int)((gx - grid_start_x) * scale);
            int screen_y = DISPLAY_HEIGHT - (int)((gy - grid_start_y + 1) * scale);
            wb_display_fill_rectangle(display, screen_x, screen_y, cell_size, cell_size);
        }
    }
    
    // Draw path - cheap early-out
    if (current_path && current_path->count > 0) {
        // set color once for path lines
        wb_display_set_color(display, 0x00FF00);
        for (int i = 0; i < current_path->count - 1; i++) {
            GridNode n1 = current_path->nodes[i];
            GridNode n2 = current_path->nodes[i + 1];
            
            int x1 = (int)((n1.x - grid_start_x) * scale);
            int y1 = DISPLAY_HEIGHT - (int)((n1.y - grid_start_y + 1) * scale);
            int x2 = (int)((n2.x - grid_start_x) * scale);
            int y2 = DISPLAY_HEIGHT - (int)((n2.y - grid_start_y + 1) * scale);
            
            wb_display_draw_line(display, x1, y1, x2, y2);
        }
        
        // draw nodes, reuse a single color change
        wb_display_set_color(display, 0x0000FF);
        for (int i = 0; i < current_path->count; i++) {
            GridNode n = current_path->nodes[i];
            int x = (int)((n.x - grid_start_x) * scale);
            int y = DISPLAY_HEIGHT - (int)((n.y - grid_start_y + 1) * scale);
            
            if (i == current_waypoint) {
                wb_display_set_color(display, 0xFF00FF);
                wb_display_fill_oval(display, x - 3, y - 3, 6, 6);
                wb_display_set_color(display, 0x0000FF); // restore
            } else {
                wb_display_draw_oval(display, x - 2, y - 2, 4, 4);
            }
        }
    }
    
    // Frontiers - update every N calls keeps same behaviour.
    static FrontierCentroid frontiers[MAX_FRONTIERS];
    static int num_frontiers = 0;
    static int frontier_update_counter = 0;
    
    frontier_update_counter++;
    if (frontier_update_counter >= 20) {
        frontier_update_counter = 0;
        num_frontiers = find_frontier_centroids(frontiers, MAX_FRONTIERS);
    }
    
    if (num_frontiers > 0) {
        double min_score = 1e9, max_score = -1e9;
        for (int i = 0; i < num_frontiers; i++) {
            if (frontiers[i].score < min_score) min_score = frontiers[i].score;
            if (frontiers[i].score > max_score) max_score = frontiers[i].score;
        }
        if (min_score > 1e9) {
            min_score = 0.0;
            max_score = 1.0;
        }
        
        // draw frontier markers
        for (int i = 0; i < num_frontiers; i++) {
            int fx = frontiers[i].x;
            int fy = frontiers[i].y;
            
            if (fx < grid_start_x || fx > grid_end_x || fy < grid_start_y || fy > grid_end_y)
                continue;
            
            int screen_x = (int)((fx - grid_start_x) * scale);
            int screen_y = DISPLAY_HEIGHT - (int)((fy - grid_start_y + 1) * scale);
            
            unsigned int f_color = frontier_score_to_color(frontiers[i].score, min_score, max_score);
            wb_display_set_color(display, f_color);
            
            int marker_half = 6;
            wb_display_fill_rectangle(display, screen_x - 1, screen_y - marker_half, 2, marker_half * 2);
            wb_display_fill_rectangle(display, screen_x - marker_half, screen_y - 1, marker_half * 2, 2);
        }
        
        // highlight best frontier (index 0)
        int fx = frontiers[0].x;
        int fy = frontiers[0].y;
        if (fx >= grid_start_x && fx <= grid_end_x && fy >= grid_start_y && fy <= grid_end_y) {
            int screen_x = (int)((fx - grid_start_x) * scale);
            int screen_y = DISPLAY_HEIGHT - (int)((fy - grid_start_y + 1) * scale);
            
            wb_display_set_color(display, 0xFFFF00);
            int thick = 2, size = 8;
            wb_display_fill_rectangle(display, screen_x - thick, screen_y - size, thick*2, size*2);
            wb_display_fill_rectangle(display, screen_x - size, screen_y - thick, size*2, thick*2);
            wb_display_draw_oval(display, screen_x - size/2, screen_y - size/2, size, size);
        }
    }
    
    // IR status - keeps existing layout but only sets font once globally
    display_ir_status(frame_count);
    
    // draw robot at center - single color calls
    int center_x = DISPLAY_WIDTH / 2;
    int center_y = DISPLAY_HEIGHT / 2;
    int robot_size = (int)(scale * 6);
    
    wb_display_set_color(display, COLOR_ROBOT);
    wb_display_fill_rectangle(display, center_x - robot_size/2, center_y - robot_size/2, 
                               robot_size, robot_size);
    
    const double *orientation = wb_supervisor_node_get_orientation(robot_node);
    double robot_theta = atan2(orientation[3], orientation[0]);
    int arrow_length = robot_size;
    int arrow_end_x = center_x + (int)(arrow_length * cos(robot_theta));
    int arrow_end_y = center_y - (int)(arrow_length * sin(robot_theta));
    
    wb_display_set_color(display, 0xFFFF00);
    wb_display_draw_line(display, center_x, center_y, arrow_end_x, arrow_end_y);
    
    // Info text - we set font once earlier
    wb_display_set_color(display, 0xFFFFFF);
    wb_display_draw_text(display, "A* Pathfinding + Frontiers", 6, 5);
    
    char info[256];
    sprintf(info, "Pos: (%d,%d)  Waypoint: %d/%d", robot_gx, robot_gy, current_waypoint, 
            current_path ? current_path->count : 0);
    wb_display_draw_text(display, info, 6, 20);
    
    sprintf(info, "Frontiers: %d", (int)num_frontiers);
    wb_display_draw_text(display, info, 6, 35);
    
    // Legend - set color then text
    int legend_x = DISPLAY_WIDTH - 110;
    int legend_y = 6;
    int box = 10;
    
    wb_display_set_color(display, cost_to_color(FREE_COST));
    wb_display_fill_rectangle(display, legend_x, legend_y, box, box);
    wb_display_set_color(display, 0xFFFFFF);
    wb_display_draw_text(display, "free", legend_x + 14, legend_y + 9);
    
    wb_display_set_color(display, cost_to_color(OBSTACLE_COST));
    wb_display_fill_rectangle(display, legend_x, legend_y + 14, box, box);
    wb_display_set_color(display, 0xFFFFFF);
    wb_display_draw_text(display, "obstacle", legend_x + 14, legend_y + 23);
    
    wb_display_set_color(display, COLOR_UNKNOWN);
    wb_display_fill_rectangle(display, legend_x, legend_y + 28, box, box);
    wb_display_set_color(display, 0xFFFFFF);
    wb_display_draw_text(display, "unknown", legend_x + 14, legend_y + 37);
    
    wb_display_set_color(display, 0xFFFF00);
    wb_display_fill_rectangle(display, legend_x, legend_y + 42, box, box);
    wb_display_set_color(display, 0xFFFFFF);
    wb_display_draw_text(display, "target", legend_x + 14, legend_y + 51);
    
    wb_display_set_color(display, 0x00FF00);
    wb_display_fill_rectangle(display, legend_x, legend_y + 56, box, box);
    wb_display_set_color(display, 0xFFFFFF);
    wb_display_draw_text(display, "path", legend_x + 14, legend_y + 65);
}
