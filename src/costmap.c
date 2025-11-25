#include "costmap.h"
#include "config.h"
#include "grid.h"
#include <math.h>

// External globals
extern unsigned int grid[GRID_SIZE][GRID_SIZE];
extern double costmap[GRID_SIZE][GRID_SIZE];

static void relax(double dist_transform[GRID_SIZE][GRID_SIZE], int x, int y, int nx, int ny, double cost) {
    double new_dist = dist_transform[ny][nx] + cost;
    if (new_dist < dist_transform[y][x]) {
        dist_transform[y][x] = new_dist;
    }
}

void generate_costmap(void) {
    static double dist_transform[GRID_SIZE][GRID_SIZE];
    const double INF = GRID_SIZE * GRID_SIZE * 2.0;
    const double SQRT2 = 1.414213562;
    const double COST_SCALING = 0.05; // tune this
    
    int x, y, k;
    
    // ---- Step 1: Initialize ----
    for (y = 0; y < GRID_SIZE; y++) {
        for (x = 0; x < GRID_SIZE; x++) {
            dist_transform[y][x] = (grid[y][x] == CELL_OBSTACLE) ? 0.0 : INF;
        }
    }
    
    // Offsets and costs (8-directions)
    const int dx8[] = {-1, 0, -1, 1, 1, 0, 1, -1};
    const int dy8[] = {0, -1, -1, -1, 0, 1, 1, 1};
    const double w8[] = {1, 1, SQRT2, SQRT2, 1, 1, SQRT2, SQRT2};
    
    // ---- Step 2: Forward pass ----
    for (y = 0; y < GRID_SIZE; y++) {
        for (x = 0; x < GRID_SIZE; x++) {
            if (dist_transform[y][x] == 0.0) continue;
            for (k = 0; k < 4; k++) {
                int nx = x + dx8[k];
                int ny = y + dy8[k];
                if (nx >= 0 && ny >= 0 && nx < GRID_SIZE && ny < GRID_SIZE) {
                    relax(dist_transform, x, y, nx, ny, w8[k]);
                }
            }
        }
    }
    
    // ---- Step 3: Backward pass ----
    for (y = GRID_SIZE - 1; y >= 0; y--) {
        for (x = GRID_SIZE - 1; x >= 0; x--) {
            if (dist_transform[y][x] == 0.0) continue;
            for (k = 4; k < 8; k++) {
                int nx = x + dx8[k];
                int ny = y + dy8[k];
                if (nx >= 0 && ny >= 0 && nx < GRID_SIZE && ny < GRID_SIZE) {
                    relax(dist_transform, x, y, nx, ny, w8[k]);
                }
            }
        }
    }
    
    // ---- Step 4: Convert to EDT-based cost map ----
    for (y = 0; y < GRID_SIZE; y++) {
        for (x = 0; x < GRID_SIZE; x++) {
            if (grid[y][x] == CELL_OBSTACLE) {
                costmap[y][x] = OBSTACLE_COST;
                continue;
            }
            if (grid[y][x] == CELL_UNKNOWN) {
                costmap[y][x] = UNKNOWN_COST;
                continue;
            }
            
            double dist = dist_transform[y][x];
            
            // No known obstacle nearby
            if (dist > INF * 0.5) {
                costmap[y][x] = FREE_COST;
            }
            // Use EDT within inflation band (INFLATION_RADIUS only limits the band, not the definition of distance)
            else if (dist <= INFLATION_RADIUS) {
                double factor = exp(-COST_SCALING * dist);
                costmap[y][x] = FREE_COST + factor * (OBSTACLE_COST - FREE_COST);
            }
            else {
                costmap[y][x] = FREE_COST;
            }
        }
    }
}
