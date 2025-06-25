#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped
from std_msgs.msg import String, Bool
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy
import random
import time
import math
from collections import deque

class ExplorationRecovery(Node):
    def __init__(self):
        super().__init__('exploration_recovery')
        
        # QoS profiles for reliable communication
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, qos_profile)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, qos_profile)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.goal_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', qos_profile)
        self.skip_pub = self.create_publisher(String, '/explore/skip_frontier', qos_profile)
        self.status_pub = self.create_publisher(String, '/exploration_recovery/status', qos_profile)
        self.recovery_active_pub = self.create_publisher(Bool, '/exploration_recovery/active', qos_profile)
        
        # State variables
        self.pose_history = deque(maxlen=20)  # Keep last 20 poses
        self.current_pose = None
        self.map_data = None
        self.last_recovery_time = time.time()
        self.recovery_attempts = 0
        self.max_recovery_attempts = 5
        self.is_recovering = False
        self.stuck_threshold_distance = 0.15  # meters
        self.stuck_threshold_time = 25.0  # seconds
        self.recovery_cooldown = 45.0  # seconds between recovery attempts
        
        # Circular motion detection
        self.orientation_history = deque(maxlen=10)
        self.total_rotation = 0.0
        
        # Recovery behavior weights (adjusted based on success)
        self.recovery_weights = {
            'backtrack': 1.0,
            'spin_and_scan': 1.0,
            'random_walk': 0.8,
            'skip_frontier': 0.6,
            'emergency_stop': 0.3
        }
        
        self.get_logger().info('🚀 Enhanced ExplorationRecovery node started.')
        self.publish_status('initialized')

    def pose_callback(self, msg):
        """Process incoming pose updates and detect stuck conditions"""
        current_time = time.time()
        pos = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Store current pose
        self.current_pose = msg
        
        # Calculate yaw from quaternion for circular motion detection
        yaw = self.quaternion_to_yaw(orientation)
        self.orientation_history.append((yaw, current_time))
        
        # Store pose with timestamp
        self.pose_history.append((pos.x, pos.y, current_time))
        
        # Detect various stuck conditions
        if not self.is_recovering and len(self.pose_history) >= 3:
            if self.detect_stuck_linear() or self.detect_stuck_circular():
                self.initiate_recovery()

    def map_callback(self, msg):
        """Store map data for path planning assistance"""
        self.map_data = msg

    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle"""
        w, x, y, z = quaternion.w, quaternion.x, quaternion.y, quaternion.z
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return yaw

    def detect_stuck_linear(self):
        """Detect if robot is stuck (not moving forward)"""
        if len(self.pose_history) < 3:
            return False
            
        # Check movement over time window
        start_pose = self.pose_history[0]
        end_pose = self.pose_history[-1]
        
        x0, y0, t0 = start_pose
        x1, y1, t1 = end_pose
        
        distance_moved = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
        time_elapsed = t1 - t0
        
        # Check if stuck based on distance and time thresholds
        is_stuck = (distance_moved < self.stuck_threshold_distance and 
                   time_elapsed > self.stuck_threshold_time and
                   (time.time() - self.last_recovery_time) > self.recovery_cooldown)
        
        if is_stuck:
            self.get_logger().warn(f'🔍 Linear stuck detected: moved {distance_moved:.3f}m in {time_elapsed:.1f}s')
        
        return is_stuck

    def detect_stuck_circular(self):
        """Detect if robot is going in circles"""
        if len(self.orientation_history) < 5:
            return False
            
        # Calculate total rotation in recent history
        total_rotation = 0.0
        for i in range(1, len(self.orientation_history)):
            prev_yaw, _ = self.orientation_history[i-1]
            curr_yaw, _ = self.orientation_history[i]
            
            # Handle angle wrapping
            diff = curr_yaw - prev_yaw
            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi
                
            total_rotation += abs(diff)
        
        # Check if robot has rotated significantly without moving
        recent_poses = list(self.pose_history)[-5:]  # Last 5 poses
        if len(recent_poses) >= 2:
            start_x, start_y, _ = recent_poses[0]
            end_x, end_y, _ = recent_poses[-1]
            linear_distance = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
            
            # Circular motion detected: significant rotation with minimal linear movement
            is_circular = (total_rotation > math.pi and linear_distance < 0.3 and
                          (time.time() - self.last_recovery_time) > self.recovery_cooldown)
            
            if is_circular:
                self.get_logger().warn(f'🔄 Circular motion detected: {total_rotation:.2f} rad rotation, {linear_distance:.2f}m displacement')
            
            return is_circular
        
        return False

    def initiate_recovery(self):
        """Start the recovery process"""
        self.get_logger().warn('🤖 Robot stuck detected! Initiating recovery sequence...')
        self.is_recovering = True
        self.recovery_attempts += 1
        self.last_recovery_time = time.time()
        
        # Publish recovery status
        self.publish_status(f'recovery_initiated_attempt_{self.recovery_attempts}')
        recovery_msg = Bool()
        recovery_msg.data = True
        self.recovery_active_pub.publish(recovery_msg)
        
        # Select and execute recovery behavior
        self.execute_recovery_behavior()

    def execute_recovery_behavior(self):
        """Choose and execute appropriate recovery behavior"""
        if self.recovery_attempts > self.max_recovery_attempts:
            self.get_logger().error('❌ Maximum recovery attempts reached. Requesting manual intervention.')
            self.publish_status('manual_intervention_required')
            self.request_manual_intervention()
            return

        # Select recovery behavior based on attempt number and weights
        behaviors = list(self.recovery_weights.keys())
        
        if self.recovery_attempts == 1:
            behavior = 'backtrack'
        elif self.recovery_attempts == 2:
            behavior = 'spin_and_scan'
        elif self.recovery_attempts == 3:
            behavior = 'random_walk'
        elif self.recovery_attempts == 4:
            behavior = 'skip_frontier'
        else:
            behavior = 'emergency_stop'
        
        self.get_logger().info(f'🔧 Executing recovery behavior: {behavior}')
        
        # Execute the selected behavior
        if behavior == 'backtrack':
            self.recovery_backtrack()
        elif behavior == 'spin_and_scan':
            self.recovery_spin_and_scan()
        elif behavior == 'random_walk':
            self.recovery_random_walk()
        elif behavior == 'skip_frontier':
            self.recovery_skip_frontier()
        elif behavior == 'emergency_stop':
            self.recovery_emergency_stop()

    def recovery_backtrack(self):
        """Backtrack to a previous known good position"""
        self.publish_status('backtracking_to_previous_position')
        
        if len(self.pose_history) < 8:
            self.get_logger().warn('⚠️ Insufficient pose history for backtracking, trying spin instead')
            self.recovery_spin_and_scan()
            return
        
        # Choose a position from earlier in the history (25% back)
        target_index = len(self.pose_history) // 4
        target_x, target_y, _ = self.pose_history[target_index]
        
        self.get_logger().info(f'🔙 Backtracking to position ({target_x:.2f}, {target_y:.2f})')
        
        # Send goal to navigation stack
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = target_x
        goal_msg.pose.position.y = target_y
        goal_msg.pose.orientation.w = 1.0  # Default orientation
        
        self.goal_pub.publish(goal_msg)
        
        # Schedule recovery completion check
        self.create_timer(10.0, self.check_recovery_success)

    def recovery_spin_and_scan(self):
        """Spin in place to scan for new paths"""
        self.publish_status('spinning_and_scanning')
        self.get_logger().info('🔄 Spinning in place to scan for new paths')
        
        # Stop current motion
        self.publish_stop_command()
        time.sleep(0.5)
        
        # Perform controlled spin
        twist = Twist()
        twist.angular.z = 0.8  # Moderate spin speed
        
        # Spin for 360 degrees (approximately)
        spin_duration = 2 * math.pi / abs(twist.angular.z)
        end_time = time.time() + spin_duration
        
        while time.time() < end_time:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop spinning
        self.publish_stop_command()
        
        # Wait a moment for sensors to process
        time.sleep(1.0)
        
        # Schedule recovery completion check
        self.create_timer(5.0, self.check_recovery_success)

    def recovery_random_walk(self):
        """Perform a short random walk to escape local minima"""
        self.publish_status('executing_random_walk')
        self.get_logger().info('🎲 Executing random walk to escape local minimum')
        
        if not self.current_pose:
            self.recovery_spin_and_scan()
            return
        
        # Generate a random nearby goal
        current_x = self.current_pose.pose.pose.position.x
        current_y = self.current_pose.pose.pose.position.y
        
        # Random offset (1-2 meters in random direction)
        angle = random.uniform(0, 2 * math.pi)
        distance = random.uniform(1.0, 2.5)
        
        target_x = current_x + distance * math.cos(angle)
        target_y = current_y + distance * math.sin(angle)
        
        self.get_logger().info(f'🎯 Random walk target: ({target_x:.2f}, {target_y:.2f})')
        
        # Send random goal
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = target_x
        goal_msg.pose.position.y = target_y
        goal_msg.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal_msg)
        
        # Schedule recovery completion check
        self.create_timer(15.0, self.check_recovery_success)

    def recovery_skip_frontier(self):
        """Skip the current unreachable frontier"""
        self.publish_status('skipping_current_frontier')
        self.get_logger().info('⏭️ Skipping current unreachable frontier')
        
        skip_msg = String()
        skip_msg.data = "skip_current"
        self.skip_pub.publish(skip_msg)
        
        # Wait and then complete recovery
        self.create_timer(3.0, self.complete_recovery)

    def recovery_emergency_stop(self):
        """Emergency stop and request manual intervention"""
        self.publish_status('emergency_stop_manual_required')
        self.get_logger().error('🛑 Emergency stop - manual intervention required')
        
        self.publish_stop_command()
        self.request_manual_intervention()

    def publish_stop_command(self):
        """Publish zero velocity to stop the robot"""
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

    def check_recovery_success(self):
        """Check if recovery was successful and complete the process"""
        if not self.is_recovering:
            return
            
        # Simple success check: if robot has moved since recovery started
        if len(self.pose_history) >= 2:
            recovery_start_pose = self.pose_history[0]
            current_pose = self.pose_history[-1]
            
            x0, y0, _ = recovery_start_pose
            x1, y1, _ = current_pose
            
            distance_since_recovery = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
            
            if distance_since_recovery > 0.5:  # Moved at least 0.5m
                self.get_logger().info(f'✅ Recovery successful! Moved {distance_since_recovery:.2f}m')
                self.complete_recovery()
                return
        
        # If recovery didn't work, try next behavior
        self.get_logger().warn('⚠️ Recovery attempt unsuccessful, trying next behavior')
        self.execute_recovery_behavior()

    def complete_recovery(self):
        """Mark recovery as complete and reset state"""
        self.is_recovering = False
        self.pose_history.clear()  # Clear history to get fresh start
        self.orientation_history.clear()
        
        self.get_logger().info('🎉 Recovery sequence completed successfully')
        self.publish_status('recovery_completed')
        
        # Publish recovery inactive status
        recovery_msg = Bool()
        recovery_msg.data = False
        self.recovery_active_pub.publish(recovery_msg)

    def request_manual_intervention(self):
        """Request manual intervention and stop autonomous operation"""
        self.publish_stop_command()
        self.recovery_attempts = self.max_recovery_attempts + 1  # Prevent further attempts
        self.is_recovering = False
        
        # Could integrate with fleet management system here
        self.get_logger().error('🆘 Manual intervention requested - autonomous recovery failed')

    def publish_status(self, status):
        """Publish current recovery status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        
    def reset_recovery_attempts(self):
        """Reset recovery attempts (can be called externally)"""
        self.recovery_attempts = 0
        self.get_logger().info('🔄 Recovery attempts counter reset')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ExplorationRecovery()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()