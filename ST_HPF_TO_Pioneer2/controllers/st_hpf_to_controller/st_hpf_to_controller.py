"""
Optimized Space-Time Hybrid Potential Field with Trajectory Optimization (ST-HPF-TO)
for Pioneer 2 Robot in Webots R2025A

Fixed Issues:
1. Proper motor velocity scaling
2. Corrected sensor readings and obstacle detection
3. Fixed coordinate system and heading calculations
4. Improved force field calculations
5. Better local minima detection and escape
6. Robust error handling
"""

import math
import numpy as np
from controller import Robot, Motor, DistanceSensor, GPS, Emitter, Receiver
import random

class ST_HPF_TO_Controller:
    def __init__(self):
        # Initialize robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Pioneer 2 specifications
        self.wheel_radius = 0.0975  # Correct Pioneer 2 wheel radius
        self.wheel_separation = 0.33  # meters
        self.max_speed = 6.28  # rad/s (max motor velocity)
        self.max_linear_speed = 0.6  # m/s
        
        # Initialize motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # Initialize GPS
        self.gps = self.robot.getDevice('gps')
        if self.gps:
            self.gps.enable(self.timestep)
        
        # Initialize distance sensors
        self.distance_sensors = []
        sensor_names = ['ds_front', 'ds_left', 'ds_right']
        for name in sensor_names:
            try:
                sensor = self.robot.getDevice(name)
                if sensor:
                    sensor.enable(self.timestep)
                    self.distance_sensors.append(sensor)
                    print(f"Initialized sensor: {name}")
            except:
                print(f"Warning: Could not initialize sensor {name}")
        
        # Initialize communication
        try:
            self.emitter = self.robot.getDevice('emitter')
            self.receiver = self.robot.getDevice('receiver')
            if self.receiver:
                self.receiver.enable(self.timestep)
        except:
            print("Warning: Communication devices not available")
            self.emitter = None
            self.receiver = None
        
        # Robot state
        self.current_position = np.array([0.0, 0.0])
        self.previous_position = np.array([0.0, 0.0])
        self.current_velocity = np.array([0.0, 0.0])
        self.current_heading = 0.0
        
        # Goal and navigation
        self.goal_position = np.array([8.0, 5.0])
        self.goal_tolerance = 0.3
        
        # Potential field parameters
        self.k_attractive = 1.5
        self.k_repulsive = 2.0
        self.d_safe = 0.4  # Safe distance from obstacles
        self.d_influence = 1.5  # Influence range
        
        # Local minima escape
        self.virtual_target = None
        self.virtual_target_active = False
        self.stuck_counter = 0
        self.stuck_threshold = 30
        self.previous_distance_to_goal = float('inf')
        self.progress_threshold = 0.05
        
        # Control parameters
        self.max_force = 5.0
        self.velocity_scale = 0.3
        
        # Obstacle data
        self.obstacles = []
        
        # Time tracking
        self.last_time = 0
        self.robot_id = random.randint(1, 100)
        
        print(f"ST-HPF-TO Controller initialized - Robot ID: {self.robot_id}")
        print(f"Goal position: {self.goal_position}")
        print(f"Number of distance sensors: {len(self.distance_sensors)}")

    def update_sensors(self):
        """Update all sensor readings"""
        # Update GPS position
        if self.gps:
            try:
                gps_values = self.gps.getValues()
                if gps_values and len(gps_values) >= 3:
                    new_position = np.array([gps_values[0], gps_values[2]])  # x, z coordinates
                    
                    # Calculate velocity
                    current_time = self.robot.getTime()
                    dt = current_time - self.last_time
                    if dt > 0:
                        self.current_velocity = (new_position - self.current_position) / dt
                    
                    self.previous_position = self.current_position.copy()
                    self.current_position = new_position
                    self.last_time = current_time
                    
            except Exception as e:
                print(f"GPS error: {e}")
        
        # Update distance sensors
        self.obstacles = []
        sensor_angles = [0, math.pi/2, -math.pi/2]  # front, left, right
        
        for i, sensor in enumerate(self.distance_sensors):
            try:
                distance = sensor.getValue()
                # Convert sensor reading to meters (assuming sensor returns mm)
                actual_distance = distance / 1000.0 if distance > 0 else float('inf')
                
                if actual_distance < self.d_influence:
                    # Calculate obstacle position in world coordinates
                    sensor_angle = self.current_heading + sensor_angles[i]
                    obstacle_x = self.current_position[0] + actual_distance * math.cos(sensor_angle)
                    obstacle_y = self.current_position[1] + actual_distance * math.sin(sensor_angle)
                    
                    self.obstacles.append({
                        'position': np.array([obstacle_x, obstacle_y]),
                        'distance': actual_distance,
                        'angle': sensor_angle
                    })
                    
            except Exception as e:
                print(f"Sensor {i} error: {e}")

    def compute_attractive_force(self):
        """Compute attractive force towards goal"""
        target = self.virtual_target if self.virtual_target_active else self.goal_position
        
        direction = target - self.current_position
        distance = np.linalg.norm(direction)
        
        if distance < 0.01:
            return np.array([0.0, 0.0])
        
        # Attractive force proportional to distance
        force_magnitude = self.k_attractive * min(distance, 2.0)  # Cap the force
        force_direction = direction / distance
        
        return force_magnitude * force_direction

    def compute_repulsive_force(self):
        """Compute repulsive force from obstacles"""
        total_force = np.array([0.0, 0.0])
        
        for obstacle in self.obstacles:
            direction = self.current_position - obstacle['position']
            distance = np.linalg.norm(direction)
            
            if distance > 0 and distance < self.d_influence:
                # Repulsive force inversely proportional to distance squared
                force_magnitude = self.k_repulsive * (1.0/distance - 1.0/self.d_influence) / (distance**2)
                force_direction = direction / distance
                total_force += force_magnitude * force_direction
        
        return total_force

    def detect_local_minimum(self):
        """Detect if robot is stuck in local minimum"""
        current_distance = np.linalg.norm(self.current_position - self.goal_position)
        
        # Check if making progress
        if abs(current_distance - self.previous_distance_to_goal) < self.progress_threshold:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
        
        self.previous_distance_to_goal = current_distance
        
        return self.stuck_counter > self.stuck_threshold

    def generate_virtual_target(self):
        """Generate virtual target to escape local minimum"""
        # Find direction perpendicular to goal direction
        goal_direction = self.goal_position - self.current_position
        goal_distance = np.linalg.norm(goal_direction)
        
        if goal_distance > 0:
            # Create perpendicular direction
            perpendicular = np.array([-goal_direction[1], goal_direction[0]])
            perpendicular = perpendicular / np.linalg.norm(perpendicular)
            
            # Add some randomness
            angle_offset = random.uniform(-math.pi/4, math.pi/4)
            rotation_matrix = np.array([[math.cos(angle_offset), -math.sin(angle_offset)],
                                      [math.sin(angle_offset), math.cos(angle_offset)]])
            
            virtual_direction = rotation_matrix @ perpendicular
            virtual_distance = random.uniform(1.0, 2.0)
            
            self.virtual_target = self.current_position + virtual_distance * virtual_direction
            self.virtual_target_active = True
            self.stuck_counter = 0
            
            print(f"Virtual target generated at: {self.virtual_target}")

    def compute_control_signals(self, force_vector):
        """Convert force vector to wheel velocities"""
        # Limit force magnitude
        force_magnitude = np.linalg.norm(force_vector)
        if force_magnitude > self.max_force:
            force_vector = force_vector / force_magnitude * self.max_force
        
        # Convert force to desired velocity
        desired_velocity = force_vector * self.velocity_scale
        
        # Limit linear velocity
        vel_magnitude = np.linalg.norm(desired_velocity)
        if vel_magnitude > self.max_linear_speed:
            desired_velocity = desired_velocity / vel_magnitude * self.max_linear_speed
        
        # Convert to robot's local frame
        cos_heading = math.cos(self.current_heading)
        sin_heading = math.sin(self.current_heading)
        
        # Desired linear and angular velocities
        desired_linear = desired_velocity[0] * cos_heading + desired_velocity[1] * sin_heading
        desired_angular = (-desired_velocity[0] * sin_heading + desired_velocity[1] * cos_heading) / 0.5
        
        # Limit angular velocity
        max_angular = 2.0
        if abs(desired_angular) > max_angular:
            desired_angular = math.copysign(max_angular, desired_angular)
        
        # Convert to wheel velocities using differential drive kinematics
        left_velocity = (desired_linear - desired_angular * self.wheel_separation / 2) / self.wheel_radius
        right_velocity = (desired_linear + desired_angular * self.wheel_separation / 2) / self.wheel_radius
        
        # Limit wheel velocities
        left_velocity = max(-self.max_speed, min(self.max_speed, left_velocity))
        right_velocity = max(-self.max_speed, min(self.max_speed, right_velocity))
        
        return left_velocity, right_velocity

    def update_heading(self, left_vel, right_vel):
        """Update robot heading based on wheel velocities"""
        dt = self.timestep / 1000.0
        
        # Calculate actual linear velocities
        left_linear = left_vel * self.wheel_radius
        right_linear = right_vel * self.wheel_radius
        
        # Calculate angular velocity
        angular_velocity = (right_linear - left_linear) / self.wheel_separation
        
        # Update heading
        self.current_heading += angular_velocity * dt
        
        # Normalize heading to [-pi, pi]
        while self.current_heading > math.pi:
            self.current_heading -= 2 * math.pi
        while self.current_heading < -math.pi:
            self.current_heading += 2 * math.pi

    def broadcast_state(self):
        """Broadcast robot state for fleet coordination"""
        if self.emitter:
            try:
                message = f"{self.robot_id},{self.current_position[0]},{self.current_position[1]},{self.robot.getTime()}"
                self.emitter.send(message.encode('utf-8'))
            except:
                pass

    def run(self):
        """Main control loop"""
        print("Starting ST-HPF-TO navigation...")
        
        while self.robot.step(self.timestep) != -1:
            # Update all sensors
            self.update_sensors()
            
            # Check if goal is reached
            distance_to_goal = np.linalg.norm(self.current_position - self.goal_position)
            if distance_to_goal < self.goal_tolerance:
                print("Goal reached!")
                self.left_motor.setVelocity(0)
                self.right_motor.setVelocity(0)
                break
            
            # Compute forces
            attractive_force = self.compute_attractive_force()
            repulsive_force = self.compute_repulsive_force()
            total_force = attractive_force + repulsive_force
            
            # Check for local minimum and handle virtual target
            if self.detect_local_minimum() and not self.virtual_target_active:
                self.generate_virtual_target()
            
            # Check if virtual target is reached
            if self.virtual_target_active:
                virtual_distance = np.linalg.norm(self.current_position - self.virtual_target)
                if virtual_distance < 0.5:
                    self.virtual_target_active = False
                    self.virtual_target = None
                    print("Virtual target reached, resuming goal navigation")
            
            # Compute and apply control signals
            left_vel, right_vel = self.compute_control_signals(total_force)
            
            # Apply velocities to motors
            self.left_motor.setVelocity(left_vel)
            self.right_motor.setVelocity(right_vel)
            
            # Update heading
            self.update_heading(left_vel, right_vel)
            
            # Broadcast state for fleet coordination
            self.broadcast_state()
            
            # Debug output every 2 seconds
            if int(self.robot.getTime()) % 2 == 0 and self.robot.getTime() % 1 < 0.1:
                print(f"Time: {self.robot.getTime():.1f}s")
                print(f"Position: ({self.current_position[0]:.2f}, {self.current_position[1]:.2f})")
                print(f"Distance to goal: {distance_to_goal:.2f}m")
                print(f"Obstacles detected: {len(self.obstacles)}")
                print(f"Motor velocities: L={left_vel:.2f}, R={right_vel:.2f}")
                print(f"Force: ({total_force[0]:.2f}, {total_force[1]:.2f})")
                print("---")

# Create and run controller
if __name__ == "__main__":
    controller = ST_HPF_TO_Controller()
    controller.run()