"""
Optimized Fleet Coordination Module for ST-HPF-TO
Handles communication and coordination between multiple Pioneer 2 robots
Fixed communication protocol and inter-robot collision avoidance
"""

import numpy as np
import math

class FleetCoordinator:
    def __init__(self, robot, robot_id):
        self.robot = robot
        self.robot_id = robot_id
        
        # Initialize communication devices
        try:
            self.emitter = robot.getDevice('emitter')
            self.receiver = robot.getDevice('receiver')
            if self.receiver:
                self.receiver.enable(robot.getBasicTimeStep())
            print(f"Fleet coordinator initialized for robot {robot_id}")
        except:
            print(f"Warning: Communication devices not available for robot {robot_id}")
            self.emitter = None
            self.receiver = None
        
        # Fleet data
        self.fleet_positions = {}
        self.fleet_timestamps = {}
        self.max_age = 5.0  # Maximum age of fleet data in seconds
        
        # Inter-robot parameters
        self.min_separation = 0.6  # Minimum distance between robots
        self.influence_radius = 2.0  # Range for inter-robot forces
        self.inter_robot_gain = 3.0  # Strength of inter-robot repulsion
    
    def broadcast_state(self, position, velocity=None, heading=0.0):
        """Broadcast robot state to fleet"""
        if not self.emitter:
            return
            
        try:
            current_time = self.robot.getTime()
            # Simple comma-separated message format
            message = f"{self.robot_id},{position[0]:.3f},{position[1]:.3f},{heading:.3f},{current_time:.3f}"
            self.emitter.send(message.encode('utf-8'))
        except Exception as e:
            print(f"Broadcast error: {e}")
    
    def receive_fleet_updates(self):
        """Receive updates from other robots"""
        if not self.receiver:
            return
            
        current_time = self.robot.getTime()
        
        # Process all messages in queue
        while self.receiver.getQueueLength() > 0:
            try:
                message = self.receiver.getData().decode('utf-8')
                parts = message.split(',')
                
                if len(parts) >= 5:
                    robot_id = int(parts[0])
                    x = float(parts[1])
                    y = float(parts[2])
                    heading = float(parts[3])
                    timestamp = float(parts[4])
                    
                    # Only store data from other robots
                    if robot_id != self.robot_id:
                        self.fleet_positions[robot_id] = np.array([x, y])
                        self.fleet_timestamps[robot_id] = timestamp
                        
            except Exception as e:
                print(f"Message parsing error: {e}")
            
            self.receiver.nextPacket()
        
        # Remove old data
        robots_to_remove = []
        for robot_id, timestamp in self.fleet_timestamps.items():
            if current_time - timestamp > self.max_age:
                robots_to_remove.append(robot_id)
        
        for robot_id in robots_to_remove:
            del self.fleet_positions[robot_id]
            del self.fleet_timestamps[robot_id]
    
    def compute_inter_robot_force(self, current_position):
        """Compute repulsive force from other robots"""
        total_force = np.array([0.0, 0.0])
        
        for robot_id, other_position in self.fleet_positions.items():
            # Calculate relative position
            relative_pos = current_position - other_position
            distance = np.linalg.norm(relative_pos)
            
            if distance > 0 and distance < self.influence_radius:
                # Stronger repulsion when closer
                if distance < self.min_separation:
                    # Emergency avoidance - very strong force
                    force_magnitude = self.inter_robot_gain * 5.0 / (distance + 0.1)
                else:
                    # Normal inter-robot repulsion
                    force_magnitude = self.inter_robot_gain / (distance**2)
                
                # Direction away from other robot
                force_direction = relative_pos / distance
                total_force += force_magnitude * force_direction
        
        return total_force
    
    def get_fleet_positions(self):
        """Get positions of all other robots"""
        return list(self.fleet_positions.values())
    
    def get_nearest_robot_distance(self, current_position):
        """Get distance to nearest robot"""
        if not self.fleet_positions:
            return float('inf')
        
        min_distance = float('inf')
        for other_position in self.fleet_positions.values():
            distance = np.linalg.norm(current_position - other_position)
            min_distance = min(min_distance, distance)
        
        return min_distance
    
    def is_path_clear(self, start_pos, end_pos):
        """Check if path between two points is clear of other robots"""
        path_vector = end_pos - start_pos
        path_length = np.linalg.norm(path_vector)
        
        if path_length == 0:
            return True
        
        path_direction = path_vector / path_length
        
        for other_position in self.fleet_positions.values():
            # Project other robot position onto path
            to_robot = other_position - start_pos
            projection_length = np.dot(to_robot, path_direction)
            
            # Check if projection is within path bounds
            if 0 <= projection_length <= path_length:
                # Calculate perpendicular distance
                projection_point = start_pos + projection_length * path_direction
                perpendicular_distance = np.linalg.norm(other_position - projection_point)
                
                # If robot is too close to path, path is not clear
                if perpendicular_distance < self.min_separation:
                    return False
        
        return True
    
    def get_fleet_status(self):
        """Get current fleet status for debugging"""
        status = {
            'active_robots': len(self.fleet_positions),
            'robot_ids': list(self.fleet_positions.keys()),
            'positions': {rid: pos.tolist() for rid, pos in self.fleet_positions.items()}
        }
        return status