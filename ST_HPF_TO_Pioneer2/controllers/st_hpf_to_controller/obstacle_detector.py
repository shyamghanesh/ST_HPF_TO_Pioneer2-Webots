"""
Optimized Obstacle Detection for Pioneer 2
Works with basic distance sensors and provides enhanced obstacle processing
"""

import math
import numpy as np

class AdvancedObstacleDetector:
    def __init__(self, robot):
        self.robot = robot
        self.distance_sensors = []
        
        # Initialize available distance sensors
        sensor_names = ['ds_front', 'ds_left', 'ds_right']
        self.sensor_angles = [0, math.pi/2, -math.pi/2]  # front, left, right
        
        for name in sensor_names:
            try:
                sensor = robot.getDevice(name)
                if sensor:
                    sensor.enable(robot.getBasicTimeStep())
                    self.distance_sensors.append(sensor)
                    print(f"Obstacle detector: initialized {name}")
            except:
                print(f"Warning: Could not initialize sensor {name}")
        
        # Obstacle processing parameters
        self.max_range = 2.0  # Maximum useful sensor range
        self.min_range = 0.1  # Minimum reliable range
        self.obstacle_threshold = 1.5  # Distance threshold for obstacle detection
        
        # Historical data for filtering
        self.sensor_history = [[] for _ in range(len(self.distance_sensors))]
        self.history_length = 5
    
    def get_raw_sensor_data(self):
        """Get raw sensor readings"""
        readings = []
        for sensor in self.distance_sensors:
            try:
                reading = sensor.getValue()
                # Convert from mm to meters (typical Webots distance sensor output)
                distance = reading / 1000.0 if reading > 0 else float('inf')
                readings.append(distance)
            except:
                readings.append(float('inf'))
        
        return readings
    
    def filter_sensor_data(self, raw_readings):
        """Apply filtering to sensor data"""
        filtered_readings = []
        
        for i, reading in enumerate(raw_readings):
            # Add to history
            if len(self.sensor_history[i]) >= self.history_length:
                self.sensor_history[i].pop(0)
            self.sensor_history[i].append(reading)
            
            # Apply median filter
            if len(self.sensor_history[i]) >= 3:
                sorted_values = sorted(self.sensor_history[i])
                filtered_reading = sorted_values[len(sorted_values)//2]
            else:
                filtered_reading = reading
            
            # Range validation
            if filtered_reading < self.min_range:
                filtered_reading = self.min_range
            elif filtered_reading > self.max_range:
                filtered_reading = self.max_range
            
            filtered_readings.append(filtered_reading)
        
        return filtered_readings
    
    def get_obstacle_data(self, robot_position, robot_heading):
        """Get comprehensive obstacle data"""
        raw_readings = self.get_raw_sensor_data()
        filtered_readings = self.filter_sensor_data(raw_readings)
        
        obstacles = []
        
        for i, distance in enumerate(filtered_readings):
            if distance < self.obstacle_threshold:
                # Calculate global angle
                global_angle = robot_heading + self.sensor_angles[i]
                
                # Calculate obstacle position in world coordinates
                obstacle_x = robot_position[0] + distance * math.cos(global_angle)
                obstacle_y = robot_position[1] + distance * math.sin(global_angle)
                
                obstacles.append({
                    'position': np.array([obstacle_x, obstacle_y]),
                    'distance': distance,
                    'angle': global_angle,
                    'sensor_id': i,
                    'relative_angle': self.sensor_angles[i]
                })
        
        return obstacles
    
    def create_local_occupancy_grid(self, obstacles, robot_position, resolution=0.1, grid_size=40):
        """Create local occupancy grid around robot"""
        # Initialize grid
        occupancy_grid = np.zeros((grid_size, grid_size))
        center = grid_size // 2
        
        # Mark obstacles in grid
        for obstacle in obstacles:
            relative_pos = obstacle['position'] - robot_position
            
            # Convert to grid coordinates
            grid_x = int(center + relative_pos[0] / resolution)
            grid_y = int(center + relative_pos[1] / resolution)
            
            # Mark obstacle and surrounding cells
            for dx in range(-1, 2):
                for dy in range(-1, 2):
                    gx, gy = grid_x + dx, grid_y + dy
                    if 0 <= gx < grid_size and 0 <= gy < grid_size:
                        # Higher values for closer obstacles
                        distance_factor = max(0.1, obstacle['distance'])
                        occupancy_grid[gx, gy] = max(occupancy_grid[gx, gy], 1.0 / distance_factor)
        
        return occupancy_grid
    
    def get_clearance_direction(self, robot_position, robot_heading, obstacles):
        """Find direction with maximum clearance"""
        # Sample directions around robot
        num_samples = 16
        max_clearance = 0
        best_direction = 0
        
        for i in range(num_samples):
            test_angle = robot_heading + (2 * math.pi * i / num_samples)
            
            # Calculate minimum distance to obstacles in this direction
            min_distance = float('inf')
            
            for obstacle in obstacles:
                # Vector from robot to obstacle
                to_obstacle = obstacle['position'] - robot_position
                obstacle_distance = np.linalg.norm(to_obstacle)
                
                if obstacle_distance > 0:
                    # Angle to obstacle
                    obstacle_angle = math.atan2(to_obstacle[1], to_obstacle[0])
                    
                    # Angular difference
                    angle_diff = abs(test_angle - obstacle_angle)
                    angle_diff = min(angle_diff, 2*math.pi - angle_diff)
                    
                    # If obstacle is roughly in this direction
                    if angle_diff < math.pi / 8:  # Within 22.5 degrees
                        min_distance = min(min_distance, obstacle_distance)
            
            # Update best direction
            if min_distance > max_clearance:
                max_clearance = min_distance
                best_direction = test_angle
        
        return best_direction, max_clearance
    
    def detect_front_obstacle(self, threshold=0.8):
        """Quick check for obstacles directly in front"""
        if len(self.distance_sensors) > 0:
            front_distance = self.get_raw_sensor_data()[0]
            return front_distance < threshold
        return False
    
    def get_side_clearances(self):
        """Get clearance on left and right sides"""
        readings = self.get_raw_sensor_data()
        
        clearances = {
            'front': readings[0] if len(readings) > 0 else float('inf'),
            'left': readings[1] if len(readings) > 1 else float('inf'),
            'right': readings[2] if len(readings) > 2 else float('inf')
        }
        
        return clearances
    
    def estimate_obstacle_velocity(self, current_obstacles, previous_obstacles, dt):
        """Estimate obstacle velocities for dynamic obstacles"""
        if not previous_obstacles or dt <= 0:
            return []
        
        obstacle_velocities = []
        
        for curr_obs in current_obstacles:
            best_match = None
            min_distance = float('inf')
            
            # Find closest previous obstacle
            for prev_obs in previous_obstacles:
                distance = np.linalg.norm(curr_obs['position'] - prev_obs['position'])
                if distance < min_distance:
                    min_distance = distance
                    best_match = prev_obs
            
            # Calculate velocity if match found and reasonable
            if best_match and min_distance < 0.5:  # Max 0.5m movement per timestep
                velocity = (curr_obs['position'] - best_match['position']) / dt
                obstacle_velocities.append({
                    'position': curr_obs['position'],
                    'velocity': velocity,
                    'distance': curr_obs['distance']
                })
        
        return obstacle_velocities