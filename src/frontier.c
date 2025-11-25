#include "frontier.h"
#include "config.h"
#include "grid.h"
#include "astar.h"
#include <webots/supervisor.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdbool.h>

// External globals
extern unsigned int grid[GRID_SIZE][GRID_SIZE];
extern unsigned char frontier_edge_map[GRID_SIZE][GRID_SIZE];
extern WbNodeRef robot_node;
extern GridPath *current_path;
extern int current_waypoint;

static inline bool is_frontier_edge(int x, int y) {
    if (grid[y][x] != CELL_FREE) return false;
    
    if ((y > 0 && grid[y-1][x] == CELL_UNKNOWN) ||
        (y < GRID_SIZE-1 && grid[y+1][x] == CELL_UNKNOWN) ||
        (x > 0 && grid[y][x-1] == CELL_UNKNOWN) ||
        (x < GRID_SIZE-1 && grid[y][x+1] == CELL_UNKNOWN)) {
        return true;
    }
    return false;
}

static void build_frontier_edge_map(void) {
    memset(frontier_edge_map, 0, sizeof(frontier_edge_map[0][0]) * GRID_SIZE * GRID_SIZE);
    
    const double *position = wb_supervisor_node_get_position(robot_node);
    double robot_x = position[0];
    double robot_y = position[1];
    
    int robot_gx, robot_gy;
    world_to_grid(robot_x, robot_y, &robot_gx, &robot_gy);
    
    int scan_radius = 150;
    int min_x = fmax(0, robot_gx - scan_radius);
    int max_x = fmin(GRID_SIZE - 1, robot_gx + scan_radius);
    int min_y = fmax(0, robot_gy - scan_radius);
    int max_y = fmin(GRID_SIZE - 1, robot_gy + scan_radius);
    
    for (int y = min_y; y <= max_y; y++) {
        for (int x = min_x; x <= max_x; x++) {
            if (is_frontier_edge(x, y)) {
                frontier_edge_map[y][x] = 1;
            }
        }
    }
}

bool has_safe_clearance(int cx, int cy, int min_clearance) {
    for (int dy = -min_clearance; dy <= min_clearance; dy++) {
        for (int dx = -min_clearance; dx <= min_clearance; dx++) {
            int check_x = cx + dx;
            int check_y = cy + dy;
            
            if (!is_valid_cell(check_x, check_y)) continue;
            if (grid[check_y][check_x] == CELL_OBSTACLE) {
                double dist = sqrt(dx * dx + dy * dy);
                if (dist <= min_clearance) {
                    return false;
                }
            }
        }
    }
    return true;
}

int find_frontier_centroids(FrontierCentroid *centroids, int max_frontiers) {
    build_frontier_edge_map();
    
    const double *position = wb_supervisor_node_get_position(robot_node);
    double robot_x = position[0];
    double robot_y = position[1];
    
    int robot_gx, robot_gy;
    world_to_grid(robot_x, robot_y, &robot_gx, &robot_gy);
    
    static bool visited[GRID_SIZE][GRID_SIZE];
    memset(visited, 0, sizeof(visited));
    
    int num_frontiers = 0;
    int scan_radius = 150;
    int min_x = fmax(0, robot_gx - scan_radius);
    int max_x = fmin(GRID_SIZE - 1, robot_gx + scan_radius);
    int min_y = fmax(0, robot_gy - scan_radius);
    int max_y = fmin(GRID_SIZE - 1, robot_gy + scan_radius);
    
    static Point queue[1000];
    
    for (int y = min_y; y <= max_y && num_frontiers < max_frontiers; y++) {
        for (int x = min_x; x <= max_x && num_frontiers < max_frontiers; x++) {
            if (frontier_edge_map[y][x] && !visited[y][x]) {
                int head = 0, tail = 0;
                int sum_x = 0, sum_y = 0, count = 0;
                
                queue[tail++] = (Point){x, y};
                visited[y][x] = true;
                
                while (head < tail && tail < 1000) {
                    Point p = queue[head++];
                    sum_x += p.x;
                    sum_y += p.y;
                    count++;
                    
                    // 8-connected neighbors
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dx = -1; dx <= 1; dx++) {
                            if (dx == 0 && dy == 0) continue;
                            
                            int nx = p.x + dx;
                            int ny = p.y + dy;
                            
                            if (nx >= min_x && nx <= max_x && ny >= min_y && ny <= max_y &&
                                !visited[ny][nx] && frontier_edge_map[ny][nx]) {
                                queue[tail++] = (Point){nx, ny};
                                visited[ny][nx] = true;
                            }
                        }
                    }
                    
                    if (tail >= 1000) break;
                }
                
                if (count >= MIN_FRONTIER_SIZE) {
                    int cx = sum_x / count;
                    int cy = sum_y / count;
                    
                    if (has_safe_clearance(cx, cy, MIN_FRONTIER_CLEARANCE)) {
                        centroids[num_frontiers].x = cx;
                        centroids[num_frontiers].y = cy;
                        centroids[num_frontiers].size = count;
                        
                        double dist = sqrt((cx - robot_gx) * (cx - robot_gx) + 
                                         (cy - robot_gy) * (cy - robot_gy));
                        centroids[num_frontiers].score = count * 2.0 / (1.0 + dist * 0.02);
                        
                        num_frontiers++;
                    }
                }
            }
        }
    }
    
    // Sort frontiers by score (bubble sort - simple and works for small arrays)
    for (int i = 0; i < num_frontiers - 1; i++) {
        for (int j = 0; j < num_frontiers - i - 1; j++) {
            if (centroids[j].score < centroids[j + 1].score) {
                FrontierCentroid temp = centroids[j];
                centroids[j] = centroids[j + 1];
                centroids[j + 1] = temp;
            }
        }
    }
    
    return num_frontiers;
}

FrontierCentroid *get_best_frontier(FrontierCentroid *frontiers, int num_frontiers) {
    return (num_frontiers > 0) ? &frontiers[0] : NULL;
}

void update_path_to_frontier(void) {
    static FrontierCentroid frontiers[MAX_FRONTIERS];
    int num_frontiers = find_frontier_centroids(frontiers, MAX_FRONTIERS);
    
    if (num_frontiers == 0) {
        printf("No frontiers found - exploration complete!\n");
        return;
    }
    
    FrontierCentroid *best_frontier = get_best_frontier(frontiers, num_frontiers);
    if (!best_frontier) return;
    
    const double *position = wb_supervisor_node_get_position(robot_node);
    int robot_gx, robot_gy;
    world_to_grid(position[0], position[1], &robot_gx, &robot_gy);
    
    if (current_path) {
        destroy_grid_path(current_path);
        current_path = NULL;
    }
    
    printf("Planning path to frontier at (%d, %d) with score %.2f\n", 
           best_frontier->x, best_frontier->y, best_frontier->score);
    
    current_path = find_path_astar(robot_gx, robot_gy, best_frontier->x, best_frontier->y);
    current_waypoint = 0;
    
    if (!current_path) {
        printf("Failed to find path to frontier\n");
    }
}
