# Swarm Drone Painting System

A multi-drone collaborative painting system where coordinated UAVs work together to create bit/pixel art on canvases. The system features precise position control and real-time coordination of multiple drones.

## Overview

This project enables autonomous drones to collaboratively paint pixel art by converting digital images into drone missions with coordinated waypoints. Each drone follows precisely planned trajectories to apply paint at specific canvas locations, creating collaborative bit art through swarm coordination.

**Key Features:**
- Multi-drone mission planning and coordination
- Image-to-dot conversion for bit/pixel art patterns
- Cascaded PID control system for precise positioning
- Real-time flight data logging and analysis
- OptiTrack motion capture integration
- Betaflight firmware integration for low-level control

### Demo Video

[![Swarm Drone Painting Demo](https://img.youtube.com/vi/4PcTPp3bISc/0.jpg)](https://www.youtube.com/watch?v=4PcTPp3bISc)

Click the image above to watch the demo on YouTube.

## Quick Start Workflow

### Example Drone Network IDs
- `pi@192.168.18.109`
- `pi@192.168.18.114`  
- `pi@192.168.18.130`

### 1. Modify PID Controller

Edit `ccodeLINUX.c` to tune PID loops as needed, then send to drones:

```bash
# Send control code to specific drone
scp ccodeLINUX.c pi@192.168.18.109:/home/pi/onboard/
```

On the **Air Traffic Controller (ATC)**, type `sleep` and then `wake` to recompile the C code on the drone.

### 2. Send Flight Mission Code

Send your user flight code (e.g., mission waypoints):

```bash
# Send flight code to drone
scp pickplacelo.py pi@192.168.18.114:/home/pi/onboard/codes
```

On the **ATC**:
- Type `update` and `pickplacelo` when prompted to update current flight code for all drones
- Or type `update 14 9` to update specific drones' user code (by last digits of IP)

### 3. Retrieve and Analyze Flight Logs

After flight, retrieve the log file:

```bash
# Download log from drone to local analyze directory
scp pi@192.168.18.114:/home/pi/onboard/ctrl_logs/2025-02-16_205325.csv ./
```

Analyze with the plotting tool:

```bash
# Example: Check velocity tracking and position for Y-axis
python3 analyse.py 2025-02-16_204633.csv -vycomp -y

# Example: Check velocity tracking and position for X-axis  
python3 analyse.py 2025-02-16_204633.csv -vxcomp -x
```

## Repository Structure

```
aerial-painting/
│
├── tuning/                 # Control system and flight infrastructure
│   ├── analyze.py         # Flight data analysis tool
│   ├── ccodeLINUX.c       # PID controller implementation
│
└── usr_codes/              # User mission files
    └── reference/         # Reference implementations
        ├── hover.py
        └── step.py
```

## Flight Data Analysis (`analyse.py`)

The `analyse.py` script is the primary tool for post-flight data analysis and visualization. It reads CSV log files and generates matplotlib plots for various control parameters.

### Basic Usage

```bash
python3 analyse.py <log_file.csv> [flags] [-s start_time] [-d end_time]
```

### Data Format

Flight logs are CSV files with the following key columns:
```
timestamp, frequency, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z,
setpoint_x, setpoint_y, setpoint_z, control_outputs, PID_terms, errors, ...
```

### Complete Flag Reference

#### Position Tracking Flags

**`-x`** - Plot X position tracking
- Shows actual position vs. desired setpoint
- Displays average position error

**`-y`** - Plot Y position tracking  
- Shows actual position vs. desired setpoint
- Displays average position error

**`-z`** - Plot Z position tracking
- Shows actual position vs. desired setpoint  
- Displays average position error

---

#### Position Controller (Outer Loop) - X Direction

**`-px`** - Position P term (X)
- Shows proportional control contribution

**`-ix`** - Position I term (X)
- Shows integral control contribution

**`-dx`** - Position D term (X)  
- Shows derivative control contribution

**`-vx`** - Desired velocity output (X)
- Commanded velocity from position controller

**`-pix`** - Position P and I terms (X)

**`-pdx`** - Position P and D terms (X)

**`-idx`** - Position I and D terms (X)

**`-pidx`** - All position PID terms (X)
- Shows P, I, D contributions overlaid
- Prints average magnitude of each term

---

#### Position Controller (Outer Loop) - Y Direction

**`-py`** - Position P term (Y)

**`-iy`** - Position I term (Y)

**`-dy`** - Position D term (Y)

**`-vy`** - Desired velocity output (Y)

**`-piy`** - Position P and I terms (Y)

**`-pdy`** - Position P and D terms (Y)

**`-idy`** - Position I and D terms (Y)

**`-pidy`** - All position PID terms (Y)
- Shows P, I, D contributions overlaid
- Prints average magnitude of each term

---

#### Velocity Controller (Inner Loop) - X Direction

**`-px2`** - Velocity P term (X)
- Inner loop proportional control

**`-ix2`** - Velocity I term (X)
- Inner loop integral control

**`-dx2`** - Velocity D term (X)
- Inner loop derivative control

**`-pix2`** - Velocity P and I terms (X)

**`-pdx2`** - Velocity P and D terms (X)

**`-idx2`** - Velocity I and D terms (X)

**`-pidx2`** - All velocity PID terms (X)
- Complete inner loop visualization
- Prints average magnitude of each term

**`-ev2`** - Velocity errors (X)
- Shows `error_vel_x` and `d_error_vel_x`
- Useful for debugging velocity tracking

---

#### Velocity Controller (Inner Loop) - Y Direction

**`-py2`** - Velocity P term (Y)

**`-iy2`** - Velocity I term (Y)

**`-dy2`** - Velocity D term (Y)

**`-piy2`** - Velocity P and I terms (Y)

**`-pdy2`** - Velocity P and D terms (Y)

**`-idy2`** - Velocity I and D terms (Y)

**`-pidy2`** - All velocity PID terms (Y)
- Complete inner loop visualization
- Prints average magnitude of each term

**`-ev2y`** - Velocity errors (Y)
- Shows `error_vel_y` and `d_error_vel_y`

---

#### Velocity Tracking Comparison

**`-vxcomp`** - **Desired vs Actual Velocity (X)** 
- Overlays pure desired velocity with actual measured velocity
- Prints average magnitudes

**`-vycomp`** - **Desired vs Actual Velocity (Y)**
- Overlays pure desired velocity with actual measured velocity
- Prints average magnitudes

<!-- ---

#### System Performance Flags

**`-f`** - Control loop frequency
- Shows frequency over time (capped display at 500 Hz)
- Prints average, max, min, and median frequency
- Helps identify timing issues

**`-m`** - Missed messages diagnostic
- Shows message count differences
- Useful for debugging communication issues

**`-i`** - Integral windup check
- Plots X, Y, Z integral accumulation
- Helps identify integral windup problems

**`-fc`** - Flight controller timing
- Time since last FC command
- Useful for debugging FC communication

**`-k`** - Kalman filter comparison (if available)
- Compares raw vs. filtered position estimates
- Shows estimation errors for X, Y, Z

---

#### Utility Flags

**`-e`** - Error mode
- Changes position plots to show error instead of tracking
- Works with `-x`, `-y`, `-z` flags

**`-s <time>`** - Start time filter
- Only analyze data after specified timestamp
- Example: `-s 5.0` starts analysis at 5 seconds

**`-d <time>`** - End time filter  
- Only analyze data before specified timestamp
- Example: `-d 30.0` ends analysis at 30 seconds

**`-c <col1> <col2> [col3]`** - Custom column plot
- Plot arbitrary CSV columns
- Specify column indices to visualize
- Optional third column for dual y-axis -->

---

### Common Usage Examples

**Basic flight analysis:**
```bash
python3 analyse.py flight_log.csv -x -y -z
```

**Tune velocity controller (X-axis):**
```bash
python3 analyse.py flight_log.csv -vxcomp -pidx2
```

**Tune position controller (Y-axis):**
```bash
python3 analyse.py flight_log.csv -y -pidy -vycomp
```

**Debug oscillations:**
```bash
python3 analyse.py flight_log.csv -pidx -pidx2 -vxcomp
```

## Mission Planning

### Bit Art / Pixel Art Missions

The project uses waypoint-based missions where each drone navigates to discrete points to create bit/pixel art:

**Example: Star Pattern** (`star*.py`)
- 5 separate mission files, one for each star edge
- Each mission contains ~10 painting dots
- Dots are created with move → descend → paint → ascend sequences
- Canvas size: 0.8m × 0.5m

---

