#ifndef GRID_H
#define GRID_H

#include <stdbool.h>
#include "config.h"

// Point structure
typedef struct {
    int x;
    int y;
} Point;

// Grid utilities
void init_grid(void);
void world_to_grid(double wx, double wy, int *gx, int *gy);
void grid_to_world(int gx, int gy, double *wx, double *wy);
int is_valid_cell(int x, int y);

// LIDAR processing
void process_lidar(void);
void draw_line_on_grid(int x0, int y0, int x1, int y1);

// Grid maintenance
void decay_counters(void);
void clear_nearby_obstacles(int inner_radius_cells, int outer_radius_cells);
void filter_connected_components(int min_size);

#endif // GRID_H
