Python 3.13.12 (tags/v3.13.12:1cbe481, Feb  3 2026, 18:22:25) [MSC v.1944 64 bit (AMD64)] on win32
Enter "help" below or click "Help" above for more information.
>>> #!/usr/bin/env python3
>>> import rospy
... import numpy as np
... import math
... 
... # ROS 消息类型
... from sensor_msgs.msg import LaserScan
... from ackermann_msgs.msg import AckermannDriveStamped
... from visualization_msgs.msg import Marker
... from geometry_msgs.msg import Pose, Point, Vector3, Quaternion, PoseWithCovarianceStamped
... from tf.transformations import euler_from_quaternion
... from std_msgs.msg import Bool # 导入 Bool 消息类型用于使能控制
... 
... # --- 可视化辅助函数 ---
... def display_direction(scale, tail, tip, idnum, frame_id="map"):
...     """
...     生成用于Rviz显示的箭头Marker。
...     """
...     m = Marker()
...     m.action = Marker.ADD
...     m.header.frame_id = frame_id
...     m.header.stamp = rospy.Time.now()
...     m.ns = "direction_arrow"
...     m.id = idnum
...     m.type = Marker.ARROW
...     m.pose.orientation.w = 1.0
...     m.scale = scale
...     m.color.r = 1.0
...     m.color.g = 0.2
...     m.color.b = 0.2
...     m.color.a = 1.0
...     m.points = [tail, tip]
...     return m
... 
... def display_threshold(scale, center_point, idnum, frame_id="map"):
...     """
    生成用于Rviz显示的圆柱体Marker，代表安全阈值范围。
    """
    m = Marker()
    m.action = Marker.ADD
    m.header.frame_id = frame_id
    m.header.stamp = rospy.Time.now()
    m.ns = "safety_radius"
    m.id = idnum
    m.type = Marker.CYLINDER
    m.pose.position = center_point
    m.pose.orientation.w = 1.0
    m.scale = scale
    m.color.r = 0.2
    m.color.g = 0.2
    m.color.b = 1.0
    m.color.a = 0.3
    return m


class FollowTheGapNode:
    def __init__(self):
        """
        节点类的构造函数
        """
        # --- ROS 参数加载 ---

        # 传感器处理参数
        [cite_start]self.scan_angle_start_deg = rospy.get_param('~scan_angle_start_deg', -100) # [cite: 73]
        [cite_start]self.scan_angle_range_deg = rospy.get_param('~scan_angle_range_deg', 200) # [cite: 74]
        [cite_start]self.obstacle_threshold = rospy.get_param('~obstacle_threshold', 2.25)     # [cite: 75]
        [cite_start]self.lidar_filter_value = rospy.get_param('~lidar_filter_value', 15.0)     # [cite: 76]

        # 算法参数
        [cite_start]self.min_gap_width_rad = rospy.get_param('~min_gap_width_rad', 0.1)       # [cite: 78]
        [cite_start]self.p_gain = rospy.get_param('~p_gain', 0.5)                             # [cite: 79]

        # 间隙选择权重参数
        [cite_start]self.width_weight = rospy.get_param('~width_weight', 0.2)                  # [cite: 81]
        [cite_start]self.current_yaw_alignment_weight = rospy.get_param('~current_yaw_alignment_weight', 0.3) # [cite: 82]
        [cite_start]self.depth_weight = rospy.get_param('~depth_weight', 0.1)                  # [cite: 83]
        [cite_start]self.yaw_alignment_std_dev = rospy.get_param('~yaw_alignment_std_dev', 0.2) # [cite: 84]

        # 全局目标航向偏好参数
        [cite_start]self.target_heading_world_deg = rospy.get_param('~target_heading_world_deg', 0.0) # [cite: 86]
        [cite_start]self.target_heading_world_rad = math.radians(self.target_heading_world_deg)      # [cite: 87]
        [cite_start]self.target_heading_weight = rospy.get_param('~target_heading_weight', 0.12)       # [cite: 88]

        # 转向平滑参数
        [cite_start]self.smoothing_factor = rospy.get_param('~smoothing_factor', 0.4)          # [cite: 90]

        # 车辆控制参数
        [cite_start]self.max_speed = rospy.get_param('~max_speed', 7.65)                        # [cite: 92]
        [cite_start]self.min_speed = rospy.get_param('~min_speed', 2.78)                        # [cite: 93]
        [cite_start]self.exp_decay_rate = rospy.get_param('~exp_decay_rate', 3.0)              # [cite: 95]
        [cite_start]self.max_steering_angle = rospy.get_param('~max_steering_angle', 0.41)     # [cite: 96]

        # 恢复行为参数
        [cite_start]self.recovery_steering_angle = rospy.get_param('~recovery_steering_angle', 0.4) # [cite: 98]
        [cite_start]self.recovery_speed = rospy.get_param('~recovery_speed', 0.0)              # [cite: 99]
        [cite_start]self.recovery_timeout = rospy.get_param('~recovery_timeout', 5.0)          # [cite: 100]
        [cite_start]self.recovery_start_time = 0.0                                             # [cite: 101]

        # 紧急避让参数
        [cite_start]self.emergency_threshold = rospy.get_param('~emergency_threshold', 0.3)    # [cite: 103]
        [cite_start]self.emergency_angle_range_deg = rospy.get_param('~emergency_angle_range_deg', 30) # [cite: 104]

        # --- 内部状态变量 ---
        [cite_start]self.current_pose = None # [cite: 106]
        [cite_start]self.current_yaw = 0.0                                                     # [cite: 107]
        [cite_start]self.is_recovering = False                                                 # [cite: 108]
        [cite_start]self.last_steering_angle = 0.0                                             # [cite: 109]
        self.is_enabled = True

        # --- ROS 发布者和订阅者 ---
        [cite_start]self.pose_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.pose_callback) # [cite: 112]
        [cite_start]self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback) # [cite: 113]

        [cite_start]self.drive_pub = rospy.Publisher('ackermann_cmd_stamped', AckermannDriveStamped, queue_size=1) # [cite: 114]

        [cite_start]self.arrow_pub = rospy.Publisher("visualization_marker/direction", Marker, queue_size=10) # [cite: 115]
        [cite_start]self.round_pub = rospy.Publisher("visualization_marker/safety_radius", Marker, queue_size=10) # [cite: 116]

        # 使能订阅者（仍然保留，可用于紧急停止或调试）
        [cite_start]self.enable_sub = rospy.Subscriber('follow_the_gap/enable', Bool, self.enable_callback) # [cite: 118]

        [cite_start]rospy.loginfo("Follow the Gap 节点已初始化完成。") # [cite: 119]
        rospy.loginfo(f"节点初始状态为 {'启用' if self.is_enabled else '禁用'}。将自动开始运行。")


    def enable_callback(self, msg):
        """
        处理使能信号的回调函数。
        """
        if self.is_enabled != msg.data:
            [cite_start]self.is_enabled = msg.data # [cite: 125]
            if self.is_enabled:
                [cite_start]rospy.loginfo("Follow the Gap: Enabled by external signal.") # [cite: 127]
            else:
                [cite_start]rospy.loginfo("Follow the Gap: Disabled by external signal. Stopping vehicle.") # [cite: 129]
                # 当禁用时，立即发送零速度指令，确保小车停止
                [cite_start]drive_msg = AckermannDriveStamped() # [cite: 131]
                [cite_start]drive_msg.header.stamp = rospy.Time.now() # [cite: 132]
                [cite_start]drive_msg.header.frame_id = "base_link" # [cite: 133]
                [cite_start]drive_msg.drive.speed = 0.0 # [cite: 134]
                [cite_start]drive_msg.drive.steering_angle = 0.0 # [cite: 135]
                [cite_start]self.drive_pub.publish(drive_msg) # [cite: 136]


    def pose_callback(self, data):
        """
        处理接收到的机器人位姿信息。
        """
        if self.current_pose is None:
             rospy.loginfo("成功接收到第一次位姿(AMCL pose)信息，导航已准备就绪。")
        [cite_start]self.current_pose = data.pose.pose # [cite: 141]

        [cite_start]orientation_q = self.current_pose.orientation # [cite: 142]
        [cite_start]orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w] # [cite: 143]
        [cite_start](roll, pitch, yaw) = euler_from_quaternion(orientation_list) # [cite: 144]
        [cite_start]self.current_yaw = yaw # [cite: 145]

    def preprocess_lidar_data(self, scan_msg):
        """
        对激光雷达数据进行预处理：裁剪、处理无效值。
        """
        [cite_start]start_angle_rad = math.radians(self.scan_angle_start_deg) # [cite: 150]
        [cite_start]end_angle_rad = math.radians(self.scan_angle_start_deg + self.scan_angle_range_deg) # [cite: 151]

        [cite_start]start_idx = int( (start_angle_rad - scan_msg.angle_min) / scan_msg.angle_increment ) # [cite: 153]
        [cite_start]end_idx = int( (end_angle_rad - scan_msg.angle_min) / scan_msg.angle_increment ) # [cite: 154]

        [cite_start]start_idx = max(0, start_idx) # [cite: 156]
        [cite_start]end_idx = min(len(scan_msg.ranges) - 1, end_idx) # [cite: 157]

        [cite_start]if start_idx > end_idx: # [cite: 159]
            [cite_start]rospy.logwarn("Scan angle range is outside valid lidar scan range. Returning empty data.") # [cite: 160]
            [cite_start]return np.array([]), 0.0, 0.0 # [cite: 161]

        [cite_start]ranges = np.array(scan_msg.ranges[start_idx:end_idx + 1]) # [cite: 162]

        [cite_start]ranges[np.isinf(ranges)] = self.lidar_filter_value # [cite: 164]
        [cite_start]ranges[np.isnan(ranges)] = self.lidar_filter_value # [cite: 165]
        [cite_start]ranges[ranges < scan_msg.range_min] = self.lidar_filter_value # [cite: 167]

        [cite_start]return ranges, start_angle_rad, scan_msg.angle_increment # [cite: 168]

    def find_gaps(self, ranges, angle_increment, current_scan_start_angle):
        """
        在处理后的激光数据中寻找所有有效的“间隙”。
        """
        [cite_start]gaps = [] # [cite: 173]
        [cite_start]in_gap = False # [cite: 174]
        [cite_start]current_gap_start_idx = -1 # [cite: 175]
        [cite_start]current_gap_max_dist = 0.0 # [cite: 176]

        [cite_start]for i in range(len(ranges)): # [cite: 177]
            [cite_start]if ranges[i] > self.obstacle_threshold: # [cite: 178]
                [cite_start]if not in_gap: # [cite: 179]
                    [cite_start]current_gap_start_idx = i # [cite: 180]
                    [cite_start]in_gap = True # [cite: 181]
                    [cite_start]current_gap_max_dist = ranges[i] # [cite: 182]
                else:
                    [cite_start]current_gap_max_dist = max(current_gap_max_dist, ranges[i]) # [cite: 184]
            else:
                [cite_start]if in_gap: # [cite: 186]
                    [cite_start]current_gap_end_idx = i - 1 # [cite: 188]
                    [cite_start]gap_start_angle_rel_lidar = current_scan_start_angle + current_gap_start_idx * angle_increment # [cite: 189]
                    [cite_start]gap_end_angle_rel_lidar = current_scan_start_angle + current_gap_end_idx * angle_increment # [cite: 190]
                    [cite_start]gap_width = abs(gap_end_angle_rel_lidar - gap_start_angle_rel_lidar) # [cite: 191]

                    [cite_start]if gap_width >= self.min_gap_width_rad: # [cite: 192]
                        gaps.append({
                            [cite_start]'start_angle_rad_rel_lidar': gap_start_angle_rel_lidar, # [cite: 194]
                            [cite_start]'end_angle_rad_rel_lidar': gap_end_angle_rel_lidar, # [cite: 195]
                            [cite_start]'center_angle_rad_rel_lidar': (gap_start_angle_rel_lidar + gap_end_angle_rel_lidar) / 2.0, # [cite: 196]
                            [cite_start]'width_rad': gap_width, # [cite: 197]
                            [cite_start]'max_dist_in_gap': current_gap_max_dist # [cite: 198]
                        })
                    [cite_start]in_gap = False # [cite: 200]
                    [cite_start]current_gap_max_dist = 0.0 # [cite: 201]

        [cite_start]if in_gap: # [cite: 203]
            [cite_start]current_gap_end_idx = len(ranges) - 1 # [cite: 204]
            [cite_start]gap_start_angle_rel_lidar = current_scan_start_angle + current_gap_start_idx * angle_increment # [cite: 205]
            [cite_start]gap_end_angle_rel_lidar = current_scan_start_angle + current_gap_end_idx * angle_increment # [cite: 206]
            [cite_start]gap_width = abs(gap_end_angle_rel_lidar - gap_start_angle_rel_lidar) # [cite: 207]
            [cite_start]if gap_width >= self.min_gap_width_rad: # [cite: 208]
                gaps.append({
                    [cite_start]'start_angle_rad_rel_lidar': gap_start_angle_rel_lidar, # [cite: 210]
                    [cite_start]'end_angle_rad_rel_lidar': gap_end_angle_rel_lidar, # [cite: 211]
                    [cite_start]'center_angle_rad_rel_lidar': (gap_start_angle_rel_lidar + gap_end_angle_rel_lidar) / 2.0, # [cite: 212]
                    [cite_start]'width_rad': gap_width, # [cite: 213]
                    [cite_start]'max_dist_in_gap': current_gap_max_dist # [cite: 214]
                })
        [cite_start]return gaps # [cite: 216]

    def select_best_gap(self, gaps):
        """
        根据多个因素选择最佳间隙。
        """
        [cite_start]if not gaps: # [cite: 221]
            [cite_start]return None # [cite: 222]

        [cite_start]best_gap_score = -1.0 # [cite: 223]
        [cite_start]best_center_angle_rel_lidar = 0.0 # [cite: 224]

        [cite_start]max_possible_distance = self.lidar_filter_value # [cite: 225]
        [cite_start]if max_possible_distance == 0: # [cite: 226]
            [cite_start]max_possible_distance = 15.0 # [cite: 227]

        [cite_start]for gap in gaps: # [cite: 228]
            [cite_start]gap_width = gap['width_rad'] # [cite: 229]
            [cite_start]gap_center_angle_rel_lidar = gap['center_angle_rad_rel_lidar'] # [cite: 230]
            [cite_start]gap_max_dist = gap['max_dist_in_gap'] # [cite: 231]

            [cite_start]current_yaw_alignment_score = math.exp(- (gap_center_angle_rel_lidar**2) / (2 * (self.yaw_alignment_std_dev)**2)) # [cite: 234]

            [cite_start]gap_center_angle_world = self.current_yaw + gap_center_angle_rel_lidar # [cite: 238]

            [cite_start]angle_diff_to_target_heading = math.atan2(math.sin(gap_center_angle_world - self.target_heading_world_rad), # [cite: 241]
                                                    [cite_start]math.cos(gap_center_angle_world - self.target_heading_world_rad)) # [cite: 242]

            [cite_start]target_heading_alignment_score = math.exp(- (angle_diff_to_target_heading**2) / (2 * (self.yaw_alignment_std_dev)**2)) # [cite: 243]

            [cite_start]depth_factor = gap_max_dist / max_possible_distance # [cite: 245]
            [cite_start]if depth_factor > 1.0: depth_factor = 1.0 # [cite: 246]

            current_gap_score = (self.width_weight * gap_width) + \
                                (self.current_yaw_alignment_weight * current_yaw_alignment_score) + \
                                (self.target_heading_weight * target_heading_alignment_score) + \
                                [cite_start](self.depth_weight * depth_factor) # [cite: 249, 250, 251, 252]

            [cite_start]if current_gap_score > best_gap_score: # [cite: 260]
                [cite_start]best_gap_score = current_gap_score # [cite: 261]
                [cite_start]best_center_angle_rel_lidar = gap_center_angle_rel_lidar # [cite: 262]

        [cite_start]return best_center_angle_rel_lidar # [cite: 263]

    def scan_callback(self, scan_msg):
        """
        核心回调函数，处理激光雷达数据并计算驾驶指令。
        """
        [cite_start]if not self.is_enabled: # [cite: 269]
            return

        # 【核心安全修复】：在执行任何操作前，必须确保已接收到有效的位姿信息
        if self.current_pose is None:
            rospy.loginfo_throttle(2, "等待AMCL的初始位姿(pose)信息...")
            # 在获得位姿前，持续发布停止指令以确保安全
            [cite_start]drive_msg = AckermannDriveStamped() # [cite: 270]
            [cite_start]drive_msg.header.stamp = rospy.Time.now() # [cite: 271]
            [cite_start]drive_msg.header.frame_id = "base_link" # [cite: 272]
            [cite_start]drive_msg.drive.speed = 0.0 # [cite: 273]
            [cite_start]drive_msg.drive.steering_angle = 0.0 # [cite: 274]
            [cite_start]self.drive_pub.publish(drive_msg) # [cite: 275]
            return

        # --- 正常逻辑 ---
        [cite_start]drive_msg = AckermannDriveStamped() # [cite: 278]
        [cite_start]drive_msg.header.stamp = rospy.Time.now() # [cite: 279]
        [cite_start]drive_msg.header.frame_id = "base_link" # [cite: 280]

        [cite_start]ranges, scan_start_angle_rad, scan_angle_increment_rad = self.preprocess_lidar_data(scan_msg) # [cite: 282]

        [cite_start]if len(ranges) == 0: # [cite: 284]
            [cite_start]rospy.logwarn("Preprocessed lidar data is empty. Stopping vehicle.") # [cite: 285]
            [cite_start]drive_msg.drive.speed = 0.0 # [cite: 286]
            [cite_start]drive_msg.drive.steering_angle = 0.0 # [cite: 287]
            [cite_start]self.drive_pub.publish(drive_msg) # [cite: 288]
            [cite_start]return # [cite: 289]

        # --- 紧急避让检测 ---
        [cite_start]emergency_start_angle_rad = 0.0 - math.radians(self.emergency_angle_range_deg / 2.0) # [cite: 291]
        [cite_start]emergency_end_angle_rad = 0.0 + math.radians(self.emergency_angle_range_deg / 2.0) # [cite: 292]

        [cite_start]e_start_idx = int((emergency_start_angle_rad - scan_start_angle_rad) / scan_angle_increment_rad) # [cite: 294]
        [cite_start]e_end_idx = int((emergency_end_angle_rad - scan_start_angle_rad) / scan_angle_increment_rad) # [cite: 295]

        [cite_start]e_start_idx = np.clip(e_start_idx, 0, len(ranges) - 1) # [cite: 297]
        [cite_start]e_end_idx = np.clip(e_end_idx, 0, len(ranges) - 1) # [cite: 298]

        if e_start_idx >= e_end_idx:
            [cite_start]min_dist_in_emergency_zone = float('inf') # [cite: 301]
        else:
            [cite_start]min_dist_in_emergency_zone = np.min(ranges[e_start_idx : e_end_idx + 1]) # [cite: 303]


        [cite_start]if min_dist_in_emergency_zone < self.emergency_threshold: # [cite: 304]
            [cite_start]rospy.logwarn(f"EMERGENCY BRAKE! Obstacle too close: {min_dist_in_emergency_zone:.2f}m. Stopping.") # [cite: 305]
            [cite_start]drive_msg.drive.speed = 0.0 # [cite: 306]
            [cite_start]drive_msg.drive.steering_angle = 0.0 # [cite: 307]
            [cite_start]self.drive_pub.publish(drive_msg) # [cite: 308]
            [cite_start]self.visualize_path(drive_msg.drive.steering_angle, drive_msg.drive.speed) # [cite: 309]
            [cite_start]self.last_steering_angle = 0.0 # [cite: 310]
            [cite_start]return # [cite: 311]

        # --- 恢复模式逻辑 ---
        [cite_start]if self.is_recovering: # [cite: 313]
            [cite_start]if rospy.Time.now().to_sec() - self.recovery_start_time > self.recovery_timeout: # [cite: 314]
                [cite_start]self.is_recovering = False # [cite: 315]
                [cite_start]rospy.loginfo("Recovery mode finished.") # [cite: 316]
                [cite_start]self.last_steering_angle = 0.0 # [cite: 317]
            else:
                [cite_start]drive_msg.drive.speed = self.recovery_speed # [cite: 319]
                [cite_start]drive_msg.drive.steering_angle = self.recovery_steering_angle # [cite: 320]
                [cite_start]self.drive_pub.publish(drive_msg) # [cite: 321]
                [cite_start]self.visualize_path(drive_msg.drive.steering_angle, drive_msg.drive.speed) # [cite: 322]
                [cite_start]self.last_steering_angle = drive_msg.drive.steering_angle # [cite: 323]
                [cite_start]return # [cite: 324]

        [cite_start]gaps = self.find_gaps(ranges, scan_angle_increment_rad, scan_start_angle_rad) # [cite: 326]

        [cite_start]target_steering_angle = 0.0 # [cite: 329]
        [cite_start]current_speed = self.max_speed # [cite: 330]

        [cite_start]best_gap_center_angle_rel_lidar = self.select_best_gap(gaps) # [cite: 332]

        [cite_start]if best_gap_center_angle_rel_lidar is not None: # [cite: 333]
            [cite_start]raw_target_steering_angle = self.p_gain * best_gap_center_angle_rel_lidar # [cite: 335]
            [cite_start]raw_target_steering_angle = np.clip(raw_target_steering_angle, -self.max_steering_angle, self.max_steering_angle) # [cite: 337]

            target_steering_angle = (1 - self.smoothing_factor) * raw_target_steering_angle + \
                                    [cite_start]self.smoothing_factor * self.last_steering_angle # [cite: 339, 340]
            [cite_start]self.last_steering_angle = target_steering_angle # [cite: 341]

            [cite_start]normalized_angle = abs(target_steering_angle) / self.max_steering_angle # [cite: 343]
            [cite_start]decay_factor = math.exp(-self.exp_decay_rate * normalized_angle) # [cite: 344]
            [cite_start]current_speed = self.min_speed + (self.max_speed - self.min_speed) * decay_factor # [cite: 345]
            [cite_start]current_speed = max(self.min_speed, current_speed) # [cite: 346]

        else:
            [cite_start]rospy.logwarn("No valid gap found! Entering recovery mode.") # [cite: 351]
            [cite_start]self.is_recovering = True # [cite: 352]
            [cite_start]self.recovery_start_time = rospy.Time.now().to_sec() # [cite: 353]
            [cite_start]target_steering_angle = self.recovery_steering_angle # [cite: 354]
            [cite_start]current_speed = self.recovery_speed # [cite: 355]
            [cite_start]self.last_steering_angle = target_steering_angle # [cite: 356]

        [cite_start]drive_msg.drive.speed = current_speed # [cite: 357]
        [cite_start]drive_msg.drive.steering_angle = target_steering_angle # [cite: 358]
        [cite_start]self.drive_pub.publish(drive_msg) # [cite: 359]

        [cite_start]self.visualize_path(target_steering_angle, current_speed) # [cite: 362]


    def visualize_path(self, steering_angle, speed):
        """
        在 Rviz 中显示机器人期望的行驶方向和安全半径。
        """
        [cite_start]if self.current_pose: # [cite: 367]
            [cite_start]arrow_tail = self.current_pose.position # [cite: 368]

            [cite_start]target_vector_x_robot = speed * math.cos(steering_angle) # [cite: 370]
            [cite_start]target_vector_y_robot = speed * math.sin(steering_angle) # [cite: 371]

            [cite_start]cos_yaw = math.cos(self.current_yaw) # [cite: 373]
            [cite_start]sin_yaw = math.sin(self.current_yaw) # [cite: 374]
            [cite_start]target_vector_x_world = target_vector_x_robot * cos_yaw - target_vector_y_robot * sin_yaw # [cite: 375]
            [cite_start]target_vector_y_world = target_vector_x_robot * sin_yaw + target_vector_y_robot * cos_yaw # [cite: 376]

            [cite_start]arrow_tip = Point() # [cite: 377]
            [cite_start]arrow_tip.x = arrow_tail.x + target_vector_x_world # [cite: 378]
            [cite_start]arrow_tip.y = arrow_tail.y + target_vector_y_world # [cite: 379]
            [cite_start]arrow_tip.z = arrow_tail.z # [cite: 380]

            [cite_start]arrow_marker = display_direction(Vector3(0.1, 0.2, 0.2), arrow_tail, arrow_tip, 0, frame_id="map") # [cite: 381]
            [cite_start]self.arrow_pub.publish(arrow_marker) # [cite: 382]

            radius_marker = display_threshold(
                [cite_start]Vector3(self.obstacle_threshold * 2, self.obstacle_threshold * 2, 0.05), # [cite: 384]
                [cite_start]self.current_pose.position, 1, frame_id="map") # [cite: 385]
            [cite_start]self.round_pub.publish(radius_marker) # [cite: 386]

# --- 主函数 ---
if __name__ == '__main__':
    try:
        [cite_start]rospy.init_node('follow_the_gap_node', anonymous=True) # [cite: 390]
        [cite_start]node = FollowTheGapNode() # [cite: 391]
        [cite_start]rospy.spin() # [cite: 392]
    [cite_start]except rospy.ROSInterruptException: # [cite: 393]
        [cite_start]rospy.loginfo("Follow the Gap node terminated.") # [cite: 394]
