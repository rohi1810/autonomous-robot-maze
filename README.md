# Autonomous Robot Maze Navigation

Webots-based autonomous exploration robot using A* path planning, frontier-based exploration, color-pillar targeting, and a safety recovery manager.

---

## Project Overview

This project implements a fully autonomous mobile robot capable of:

- Building a 2D occupancy grid map using LiDAR
- Exploring unknown environments via frontier-based exploration
- Detecting and navigating to a blue pillar, then a yellow pillar
- Planning optimal paths using A*
- Avoiding collisions using a safety manager with recovery behavior

The system runs entirely inside Webots and integrates perception, mapping, planning, control, and safety into a unified architecture.

---

## Core Features

### Occupancy Grid Mapping
- 2D grid-based environment representation
- LiDAR ray integration with Bresenham-style tracing
- Temporal filtering (free/obstacle counters)
- Noise decay and small-blob removal
- Pillar cells protected from LiDAR overwrite

### A* Path Planning
- Custom standalone A* implementation
- 8-connected grid expansion
- Euclidean heuristic
- Early-exit protection to prevent long stalls
- Waypoint generation for navigation

### Frontier-Based Exploration
- Frontier detection (free cells adjacent to unknown cells)
- Frontier clustering and centroid computation
- Candidate scoring by size and distance
- Failed frontier cache to prevent repeated unreachable attempts

### Adaptive Cost Map & Obstacle Inflation
- Distance transform-based inflation
- High cost near obstacles
- Medium cost for unknown cells
- Adaptive inflation:
  - Smaller in narrow corridors
  - Larger in open space

### Camera-Based Perception
- RGB blob detection (green/red)
- Pixel → bearing → world → grid projection
- Median depth filtering (3×3)
- Red wall inward segment marking
- Fallback distance handling when depth fails

### Pillar Detection & Targeting
- Blue pillar detected and marked first
- Yellow pillar detected afterward
- Multi-sample averaging for stable world coordinates
- Grid disk painting for pillar marking
- Safe handling of depth instability

### Navigation & Control
- Waypoint-based path execution
- PID rotation and translation
- Heading hysteresis to prevent oscillation
- Slew rate limiter for smooth motion
- Periodic replanning

### Safety Manager
- IR-based obstacle detection
- Pitch monitoring for instability
- Velocity scaling
- Hard stop override
- Recovery sequence:
  - Stop
  - Backup
  - Turn away
  - Resume after clearance
- Virtual obstacle marking to prevent repeated collisions

### Visualization & Debugging
- Occupancy grid rendering
- Cost heatmap display
- Planned path visualization
- Frontier highlighting
- Safety and IR status display

---

## System Architecture

Main control loop:

1. Safety update  
2. LiDAR mapping update  
3. Camera perception update  
4. Cost map generation  
5. Visualization  
6. State machine decision  

State machine modes:
- Frontier exploration
- Blue pillar navigation
- Yellow pillar navigation
- Safety recovery

---

## Mathematical Foundations

### Pixel to Bearing
β(x) = −(x − w/2) · (FOVh / w)

### Bearing to World
α = θ + β  
wx = rx + d · cos(α)  
wy = ry + d · sin(α)

### World to Grid
gx = floor(wx / RES + N/2)  
gy = floor(wy / RES + N/2)

### Adaptive Inflation
Traversal cost increases as distance to nearest obstacle decreases.  
Inflation radius adapts dynamically based on local clearance.

---

## Technologies Used

- Webots
- C
- LiDAR integration
- RGB + Depth camera processing
- A* path planning
- PID control
- Occupancy grid mapping
- Frontier-based exploration

---

## How to Run

1. Open the project in Webots  
2. Load the appropriate world file  
3. Build the controller  
4. Run the simulation  

The robot will:
- Explore the maze
- Map the environment
- Detect and navigate to the blue pillar
- Then detect and navigate to the yellow pillar

---

## Engineering Challenges Addressed

- Preventing mapping corruption during unstable motion  
- Avoiding infinite frontier replanning  
- Handling depth sensor failure at close range  
- Adaptive inflation to prevent corridor deadlocks  
- Integrating safety overrides without breaking navigation logic  

---

## Future Improvements

- Dynamic obstacle handling  
- SLAM-based pose correction  
- Multi-goal optimization  
- Performance profiling for large-scale grids  


