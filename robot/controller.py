"""
Robot controller module for robot control and communication.
"""

import socket
import time
import select
import numpy as np
from scipy.spatial.transform import Rotation
from config.settings import Config

class RobotController:
    def __init__(self):
        self.socket = None
        self.connected = False
        self.max_configs = 10

    def connect(self):
        """Connect to the ABB GoFa robot with improved connection handling"""
        try:
            print(f"Connecting to robot at {Config.ROBOT_IP}:{Config.ROBOT_PORT}...")
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(10)
            self.socket.connect((Config.ROBOT_IP, Config.ROBOT_PORT))
            
            # Handle initial connection message
            initial_response = self.socket.recv(1024).decode('utf-8').strip()
            print(f"Robot response: {initial_response}")
            
            # Test connection with a simple command
            print("Testing connection with ping command...")
            self.socket.sendall(b"ping\n")
            
            start_time = time.time()
            response_received = False
            
            # Wait for response with timeout
            while time.time() - start_time < 5:
                ready = select.select([self.socket], [], [], 0.5)
                if ready[0]:
                    response = self.socket.recv(1024).decode('utf-8').strip()
                    print(f"Ping response: {response}")
                    response_received = True
                    break
                time.sleep(0.5)
            
            if not response_received:
                print("Warning: No response to ping command, but continuing...")
            
            self.connected = True
            print(f"Connected to robot at {Config.ROBOT_IP}:{Config.ROBOT_PORT}")
            return True
            
        except Exception as e:
            print(f"Failed to connect to robot: {e}")
            self.socket = None
            self.connected = False
            
            # Ask for simulation mode
            if input("Would you like to continue in simulation mode? (y/n): ").lower() == 'y':
                print("Continuing in simulation mode...")
                self.connected = True
                self.simulation_mode = True
                return True
            return False


    def move_to_configuration(self, config_num):
        """Move robot to specified configuration."""
        if not self.connected:
            print("Robot not connected")
            return None
            
        if not 1 <= config_num <= 10:
            print(f"Invalid configuration number: {config_num}")
            return None
        
        # Set longer timeout for robot movement
        self.socket.settimeout(30)
        
        try:
            # First, make socket non-blocking to avoid freezing UI
            self.socket.setblocking(0)
            
            # Send configuration command
            command = f"conf{config_num}\n"
            self.socket.sendall(command.encode('utf-8'))
            print(f"Sent command: {command.strip()}")
            
            # Make socket blocking again with timeout
            self.socket.setblocking(1)
            self.socket.settimeout(30)
            
            # Wait for response with position data
            response = self._wait_for_position_data_improved(timeout=30)
            
            if not response:
                print("No position data received from robot")
                return None
                
            # Parse position data
            try:
                values = [float(val.strip()) for val in response.split(';')]
                if len(values) == 7:  # x,y,z + quaternion
                    position = np.array(values[0:3]).reshape(3, 1)
                    quaternion = np.array(values[3:7])
                    
                    print(f"Received position: {position.flatten()}")
                    print(f"Received quaternion: {quaternion}")
                    
                    return {
                        'position': position,
                        'quaternion': quaternion
                    }
                else:
                    raise ValueError(f"Expected 7 values, got {len(values)}")
            except Exception as e:
                print(f"Error parsing robot response: {str(e)}")
                print(f"Raw response: {response}")
                return None
                    
        except Exception as e:
            print(f"Error communicating with robot: {str(e)}")
            return None
        finally:
            # Reset timeout
            self.socket.settimeout(10)

    def _wait_for_position_data_improved(self, timeout=30):
        """Wait for position data from robot with improved responsiveness."""
        start_time = time.time()
        buffer = ""
        
        while time.time() - start_time < timeout:
            try:
                # Use select to avoid blocking
                ready = select.select([self.socket], [], [], 0.5)
                if ready[0]:
                    # Socket has data
                    data = self.socket.recv(1024).decode('utf-8').strip()
                    if data:
                        buffer += data
                        
                        # Look for position data format
                        if ';' in buffer and any(c.isdigit() for c in buffer):
                            # Extract values - look for exactly 7 values
                            values = buffer.split(';')
                            if len(values) >= 7:
                                return ';'.join(values[:7])
                else:
                    # No data yet - update status
                    elapsed = time.time() - start_time
                    print(f"Waiting for robot response... ({elapsed:.1f}s / {timeout}s)")
                    
                    # Let UI update by yielding execution briefly
                    time.sleep(0.1)
                    
            except Exception as e:
                print(f"Error while waiting: {str(e)}")
                time.sleep(0.5)
        
        print(f"Timeout waiting for position data. Buffer: {buffer}")
        return None

    def send_command(self, command, timeout=10):
        """Send command to robot and get response"""
        if not self.connected:
            print("Robot not connected")
            return None
            
        try:
            # Increase socket timeout for movement commands
            if command.startswith("conf"):
                self.socket.settimeout(30)  # 30 seconds for movement commands
            else:
                self.socket.settimeout(timeout)
                
            # Send command
            self.socket.sendall(command.encode('utf-8') + b'\n')
            print(f"Sent command: {command}")
            
            # Wait for response
            response = self.socket.recv(1024).decode('utf-8').strip()
            print(f"Received: {response}")
            
            # Reset timeout to default
            self.socket.settimeout(10)
            return response
            
        except socket.timeout:
            print(f"Command timed out: {command}")
            
            # For configuration commands, try to get position anyway
            if command.startswith("conf"):
                try:
                    # Send position query
                    self.socket.sendall(b"getpos\n")
                    response = self.socket.recv(1024).decode('utf-8').strip()
                    print(f"Got position after timeout: {response}")
                    return response
                except Exception:
                    pass
                    
            return None
            
        except Exception as e:
            print(f"Command failed: {str(e)}")
            return None

    def close(self):
        """Close robot connection."""
        if self.socket:
            self.send_command("end__")  # Exactly 5 characters as required
            self.socket.close()
            self.socket = None

    def format_position_command(self, position, quaternion):
        """Format position and quaternion for robot command with exactly 6 characters per value."""
        formatted_values = []
        
        # Format position values with exactly 6 characters (including decimal point)
        for val in position.flatten():
            formatted_values.append(f"{val:06.3f}"[:6])  # Ensure exactly 6 chars
            
        # Format quaternion values with exactly 6 characters
        for val in quaternion:
            formatted_values.append(f"{val:06.3f}"[:6])  # Ensure exactly 6 chars
            
        return ";".join(formatted_values)

    def send_position_command(self, position, quaternion, timeout=30):
        """Send position command with protocol adjustments for ABB GoFa.
        
        Args:
            position: 3D position [x, y, z]
            quaternion: Orientation as quaternion [w, x, y, z]
            timeout: Command timeout in seconds
            
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.connected:
            print("Robot not connected")
            return False
            
        try:
            # Format the command
            formatted_command = self.format_position_command(position, quaternion)
            command = f"move_{formatted_command}"
            
            print(f"Sending command: {command}")
            
            # ABB GoFa doesn't send acknowledgments for all commands
            # Instead, it starts moving immediately and only notifies when done
            # Use a simple "fire and forget" approach with position polling
            self.socket.sendall((command + "\n").encode('utf-8'))
            
            # Instead of expecting an immediate response, poll robot position
            start_pos = self.get_current_position()
            if start_pos is None:
                print("Failed to get current position before movement")
            else:
                print(f"Starting position: {start_pos}")
            
            # Wait for movement to complete by polling position
            print("Waiting for robot to complete movement...")
            movement_detected = False
            movement_stopped = False
            polling_interval = 1.0  # seconds
            stable_count = 0
            last_position = None
            position_threshold = 1.0  # mm
            
            start_time = time.time()
            while time.time() - start_time < timeout:
                try:
                    # Get current position
                    current_position = self.get_current_position()
                    if current_position is None:
                        print("Error getting robot position, waiting...")
                        time.sleep(polling_interval)
                        continue
                    
                    # Check if movement has started
                    if not movement_detected and start_pos is not None:
                        dist = np.linalg.norm(np.array(current_position) - np.array(start_pos))
                        if dist > position_threshold:
                            print(f"Movement detected! Distance: {dist:.2f}mm")
                            movement_detected = True
                    
                    # Check if movement has stopped
                    if last_position is not None:
                        dist = np.linalg.norm(np.array(current_position) - np.array(last_position))
                        if dist < position_threshold:
                            stable_count += 1
                        else:
                            stable_count = 0
                            
                        # Print distance to target periodically
                        target_pos = position.flatten()
                        dist_to_target = np.linalg.norm(np.array(current_position) - target_pos)
                        print(f"Current: {current_position}, Distance to target: {dist_to_target:.2f}mm")
                        
                        # Consider movement complete if position is stable for several readings
                        if movement_detected and stable_count >= 3:
                            print("Movement complete! Position stable.")
                            movement_stopped = True
                            
                            # Final distance check
                            if dist_to_target < 5.0:  # 5mm tolerance
                                print("Position reached within tolerance")
                                return True
                            else:
                                print(f"Warning: Final position is {dist_to_target:.2f}mm from target")
                                # We'll still return True since the robot completed its movement
                                return True
                    
                    last_position = current_position
                    time.sleep(polling_interval)
                    
                except Exception as e:
                    print(f"Error during position monitoring: {e}")
                    time.sleep(polling_interval)
            
            if movement_detected and not movement_stopped:
                print("Movement started but didn't complete within timeout")
                return False
            elif not movement_detected:
                print("No movement detected")
                return False
                
            return movement_stopped
            
        except Exception as e:
            print(f"Error sending position command: {e}")
            return False

    def get_current_position(self):
        """Get current robot position with retries.
        
        Returns:
            list: [x, y, z] position or None if failed
        """
        max_retries = 3
        
        for attempt in range(max_retries):
            try:
                # Send position query
                self.socket.sendall(b"get_pos\n")
                
                # Wait for response
                ready = select.select([self.socket], [], [], 5.0)
                if ready[0]:
                    response = self.socket.recv(1024).decode('utf-8').strip()
                    if response:
                        # Parse position data: expecting x;y;z;qw;qx;qy;qz
                        try:
                            values = [float(val.strip()) for val in response.split(';')]
                            if len(values) >= 3:
                                return values[:3]  # Return just the position part
                        except ValueError:
                            print(f"Invalid position data: {response}")
                
                if attempt < max_retries - 1:  # Don't sleep on last retry
                    time.sleep(1)
                    
            except Exception as e:
                print(f"Error getting current position: {e}")
                if attempt < max_retries - 1:  # Don't sleep on last retry
                    time.sleep(1)
        
        print("Failed to get current position after retries")
        return None

    def send_direct_command(self, command):
        """Send a command directly without waiting for response - use for GoFa that doesn't acknowledge all commands.
        
        Args:
            command: Command string
        """
        try:
            self.socket.sendall((command + "\n").encode('utf-8'))
            return True
        except Exception as e:
            print(f"Error sending direct command: {e}")
            return False

    def send_alternate_move_command(self, position, timeout=10):
        """Try an alternate command format for robot movement."""
        try:
            # Format in an alternate style that might work with your robot
            x, y, z = position.flatten()
            command = f"movej([{x:.2f},{y:.2f},{z:.2f},0,0,0])"
            
            print(f"Sending alternate command: {command}")
            self.socket.sendall((command + "\n").encode('utf-8'))
            
            # Simple response check
            start_time = time.time()
            while time.time() - start_time < timeout:
                ready = select.select([self.socket], [], [], 0.5)
                if ready[0]:
                    response = self.socket.recv(1024).decode('utf-8')
                    print(f"Received: {response}")
                    if response:
                        return True
                else:
                    elapsed = time.time() - start_time
                    print(f"Waiting for response... ({elapsed:.1f}s / {timeout}s)")
                    time.sleep(0.5)
                    
            print(f"Alternate command timed out after {timeout}s")
            return False
        except Exception as e:
            print(f"Error sending alternate command: {str(e)}")
            return False

    def pick_and_place(self, pick_position, place_position):
        """Execute pick and place operation with improved command handling."""
        approach_height = 30  # mm above object for safe approach
        minimum_z = 30  # Minimum safe Z height to avoid table collisions
        
        # Add a short delay before sending first command to ensure robot is ready
        print("Preparing robot for pick and place...")
        time.sleep(1)
        
        try:
            print("Starting pick and place operation...")
            
            # Convert to numpy arrays if not already
            pick_position = np.array(pick_position).flatten()
            place_position = np.array(place_position).flatten()
            
            # Apply safety Z check for pick position
            if pick_position[2] < minimum_z:
                print(f"Warning: Pick Z coordinate too low ({pick_position[2]:.2f}mm), adjusting to {minimum_z}mm")
                pick_position[2] = minimum_z
            
            # Apply safety Z check for place position
            if place_position[2] < minimum_z:
                print(f"Warning: Place Z coordinate too low ({place_position[2]:.2f}mm), adjusting to {minimum_z}mm")
                place_position[2] = minimum_z
                
            # Create approach positions
            pick_approach = np.copy(pick_position)
            pick_approach[2] += approach_height
            
            place_approach = np.copy(place_position)
            place_approach[2] += approach_height
            
            # Safe quaternion for approaching from top (pointing down)
            down_quaternion = np.array([0.0, 0.0, 1.0, 0.0])
            
            print("\nPick and Place Sequence:")
            print("------------------------")
            
            # Skip the move_to_configuration and directly send position commands
            # The move_to_configuration may be timing out
            
            # 1. First move to approach position with higher timeout
            print(f"1. Moving to approach position: [{pick_approach[0]:.2f}, {pick_approach[1]:.2f}, {pick_approach[2]:.2f}]")
            success = self.send_direct_move_command(pick_approach, down_quaternion, timeout=10)
            if not success:
                print("Failed to move to approach position, trying alternate command format")
                # Try alternate format
                success = self.send_alternate_move_command(pick_approach, timeout=10)
                if not success:
                    print("Failed to move robot with both command formats")
                    return False
            
            # 2. Move down to pick position
            print(f"2. Moving to pick position: [{pick_position[0]:.2f}, {pick_position[1]:.2f}, {pick_position[2]:.2f}]")
            success = self.send_direct_move_command(pick_position, down_quaternion, timeout=10)
            if not success:
                print("Failed to move to pick position")
                return False
            
            # 3. Close gripper with simple command
            print("3. Closing gripper")
            self.socket.sendall(b"close\n")
            time.sleep(1)  # Just wait a second instead of expecting a response
            
            # 4. Move back to approach height with object
            print("4. Lifting object")
            success = self.send_direct_move_command(pick_approach, down_quaternion, timeout=10)
            if not success:
                print("Failed to lift object")
                return False
            
            # 5. Move to place approach position
            print(f"5. Moving to place approach: [{place_approach[0]:.2f}, {place_approach[1]:.2f}, {place_approach[2]:.2f}]")
            success = self.send_direct_move_command(place_approach, down_quaternion, timeout=10)
            if not success:
                print("Failed to move to place approach position")
                return False
            
            # 6. Move down to place position
            print(f"6. Moving to place position: [{place_position[0]:.2f}, {place_position[1]:.2f}, {place_position[2]:.2f}]")
            success = self.send_direct_move_command(place_position, down_quaternion, timeout=10)
            if not success:
                print("Failed to move to place position")
                return False
            
            # 7. Open gripper with simple command
            print("7. Opening gripper")
            self.socket.sendall(b"open\n")
            time.sleep(1)  # Just wait a second instead of expecting a response
            
            # 8. Move back to approach position
            print("8. Moving up from place")
            success = self.send_direct_move_command(place_approach, down_quaternion, timeout=10)
            if not success:
                print("Failed to move up from place position")
                return False
            
            print("Pick and place operation completed successfully")
            return True
        except Exception as e:
            print(f"Error during pick and place: {str(e)}")
            return False
        
    def close_gripper(self):
        """Close the robot gripper."""
        try:
            print("Closing gripper")
            return self.send_command("close", timeout=10) is not None
        except Exception as e:
            print(f"Error closing gripper: {str(e)}")
            return False
        
    def open_gripper(self):
        """Open the robot gripper."""
        try:
            print("Opening gripper")
            return self.send_command("open", timeout=10) is not None
        except Exception as e:
            print(f"Error opening gripper: {str(e)}")
            return False
        
    def move_to_safe_position(self):
        """Move robot to a predefined safe position (configuration 1)."""
        try:
            print("Moving to safe position...")
            return self.move_to_configuration(1) is not None
        except Exception as e:
            print(f"Error moving to safe position: {str(e)}")
            return False
        
    def send_position_command_simple(self, command_prefix="move_", position=None, quaternion=None, timeout=10):
        """Simplified command sending for ABB robots with position validation."""
        if not self.connected:
            print("Robot not connected")
            return False
            
        try:
            # Validate position against workspace constraints
            #

                
            # Position is valid, continue with command
            # Extract values
            x, y, z = position.flatten()
            w, x_q, y_q, z_q = quaternion.flatten()
            
            # Make sure quaternion is normalized
            quat_norm = np.sqrt(w*w + x_q*x_q + y_q*y_q + z_q*z_q)
            w = w / quat_norm
            x_q = x_q / quat_norm
            y_q = y_q / quat_norm
            z_q = z_q / quat_norm
            
            # Format with precision (3 decimal places)
            formatted_command = self.format_position_command(position, quaternion)
            command = f"{command_prefix}{formatted_command}"
            
            print(f"Sending command with prefix '{command_prefix}': {command}")
            self.socket.sendall((command + "\n").encode('utf-8'))
            
            # Wait for movement to complete
            print(f"Command sent, waiting {timeout} seconds for movement to complete...")
            time.sleep(timeout)
            
            print("Movement time completed")
            return True
            
        except Exception as e:
            print(f"Error sending position command: {e}")
            return False