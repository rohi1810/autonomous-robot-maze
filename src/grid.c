#include "grid.h"
#include "config.h"
#include <webots/lidar.h>
#include <webots/supervisor.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

// External globals
extern unsigned int grid[GRID_SIZE][GRID_SIZE];
extern unsigned int obstacle_counter[GRID_SIZE][GRID_SIZE];
extern unsigned int free_counter[GRID_SIZE][GRID_SIZE];
extern WbDeviceTag lidar;
extern WbNodeRef robot_node;

void init_grid(void) {
    memset(grid, CELL_UNKNOWN, sizeof(grid));
    memset(obstacle_counter, 0, sizeof(obstacle_counter));
    memset(free_counter, 0, sizeof(free_counter));
}

void world_to_grid(double wx, double wy, int *gx, int *gy) {
    *gx = (int)(wx / GRID_RESOLUTION) + GRID_SIZE / 2;
    *gy = (int)(wy / GRID_RESOLUTION) + GRID_SIZE / 2;
}

void grid_to_world(int gx, int gy, double *wx, double *wy) {
    *wx = (double)(gx - GRID_SIZE / 2) * GRID_RESOLUTION;
    *wy = (double)(gy - GRID_SIZE / 2) * GRID_RESOLUTION;
}

int is_valid_cell(int x, int y) {
    return (x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE);
}

void draw_line_on_grid(int x0, int y0, int x1, int y1) {
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    while (1) {
        if (is_valid_cell(x0, y0)) {
            if (x0 != x1 || y0 != y1) {
                // Free space along ray
                if (grid[y0][x0] != CELL_OBSTACLE) {
                    free_counter[y0][x0]++;
                    if (free_counter[y0][x0] >= FREE_THRESHOLD) {
                        grid[y0][x0] = CELL_FREE;
                        obstacle_counter[y0][x0] = 0;
                    }
                }
            } else {
                // Obstacle at endpoint
                obstacle_counter[y0][x0]++;
                if (obstacle_counter[y0][x0] >= OBSTACLE_THRESHOLD) {
                    grid[y0][x0] = CELL_OBSTACLE;
                    free_counter[y0][x0] = 0;
                }
            }
        }
        
        if (x0 == x1 && y0 == y1) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void clear_nearby_obstacles(int inner_radius_cells, int outer_radius_cells) {
    const double *position = wb_supervisor_node_get_position(robot_node);
    double robot_x = position[0];
    double robot_y = position[1];
    
    int robot_gx, robot_gy;
    world_to_grid(robot_x, robot_y, &robot_gx, &robot_gy);
    
    for (int dy = -outer_radius_cells; dy <= outer_radius_cells; dy++) {
        for (int dx = -outer_radius_cells; dx <= outer_radius_cells; dx++) {
            int gx = robot_gx + dx;
            int gy = robot_gy + dy;
            
            if (!is_valid_cell(gx, gy)) continue;
            
            int dist_sq = dx * dx + dy * dy;
            if (dist_sq > outer_radius_cells * outer_radius_cells) continue;
            if (dist_sq <= inner_radius_cells * inner_radius_cells) continue;
            
            if (grid[gy][gx] == CELL_OBSTACLE) {
                grid[gy][gx] = CELL_UNKNOWN;
            }
        }
    }
}

void filter_connected_components(int min_size) {
    static bool visited[GRID_SIZE][GRID_SIZE];
    memset(visited, 0, sizeof(visited));
    
    int dx8[] = {1, -1, 0, 0, 1, -1, 1, -1};
    int dy8[] = {0, 0, 1, -1, 1, 1, -1, -1};
    
    Point queue[MAX_QUEUE];
    
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            if (grid[y][x] == CELL_OBSTACLE && !visited[y][x]) {
                int head = 0, tail = 0, count = 0;
                Point component[MAX_QUEUE];
                
                queue[tail++] = (Point){x, y};
                visited[y][x] = true;
                
                while (head < tail) {
                    Point p = queue[head++];
                    component[count++] = p;
                    
                    for (int i = 0; i < 8; i++) {
                        int nx = p.x + dx8[i];
                        int ny = p.y + dy8[i];
                        
                        if (nx < 0 || ny < 0 || nx >= GRID_SIZE || ny >= GRID_SIZE) continue;
                        if (!visited[ny][nx] && grid[ny][nx] == CELL_OBSTACLE) {
                            queue[tail++] = (Point){nx, ny};
                            visited[ny][nx] = true;
                        }
                    }
                }
                
                // Remove small components
                if (count < min_size) {
                    for (int i = 0; i < count; i++) {
                        grid[component[i].y][component[i].x] = CELL_FREE;
                        obstacle_counter[component[i].y][component[i].x] = 0;
                    }
                }
            }
        }
    }
}

void process_lidar(void) {
    const double *position = wb_supervisor_node_get_position(robot_node);
    const double *orientation = wb_supervisor_node_get_orientation(robot_node);
    
    double robot_x = position[0];
    double robot_y = position[1];
    double robot_theta = atan2(orientation[3], orientation[0]);
    
    int robot_gx, robot_gy;
    world_to_grid(robot_x, robot_y, &robot_gx, &robot_gy);
    
    const float *ranges = wb_lidar_get_range_image(lidar);
    int resolution = wb_lidar_get_horizontal_resolution(lidar);
    double fov = wb_lidar_get_fov(lidar);
    double max_range = wb_lidar_get_max_range(lidar);
    
    // Process every 2nd ray for performance
    for (int i = 0; i < resolution; i += 2) {
        double range = ranges[i];
        if (range < 0.05 || range > max_range * 0.95) continue;
        
        double ray_angle = fov / 2 - (double)i * fov / resolution;
        double world_angle = robot_theta + ray_angle;
        
        double end_x = robot_x + range * cos(world_angle);
        double end_y = robot_y + range * sin(world_angle);
        
        int end_gx, end_gy;
        world_to_grid(end_x, end_y, &end_gx, &end_gy);
        
        draw_line_on_grid(robot_gx, robot_gy, end_gx, end_gy);
    }
    
    // Mark robot position
    if (is_valid_cell(robot_gx, robot_gy)) {
        grid[robot_gy][robot_gx] = CELL_ROBOT;
    }
}

void decay_counters(void) {
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            // Decay obstacle counters
            if (grid[y][x] != CELL_OBSTACLE && obstacle_counter[y][x] > 0) {
                obstacle_counter[y][x] -= COUNTER_DECAY;
                if (obstacle_counter[y][x] <= 0) {
                    grid[y][x] = CELL_UNKNOWN;
                }
            }
            
            // Decay free counters
            if (grid[y][x] != CELL_FREE && free_counter[y][x] > 0) {
                free_counter[y][x] -= COUNTER_DECAY;
                if (free_counter[y][x] <= 0) {
                    grid[y][x] = CELL_UNKNOWN;
                }
            }
        }
    }
}
