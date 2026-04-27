#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import sys
import numpy as np

class SimpleObstacleDetector(Node):
    def __init__(self, center_angle=0.0, range_width=30.0, reliability_threshold=0.6):
        super().__init__('simple_obstacle_detector')
        
        # 检测范围：center_angle ± range_width/2
        self.center_angle = center_angle
        self.range_width = range_width
        
        # 可靠性阈值 (0-1)
        self.reliability_threshold = reliability_threshold
        
        # 订阅激光雷达
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.get_logger().info(f'避障检测器已启动 - 范围: {center_angle-range_width/2:.0f}° ~ {center_angle+range_width/2:.0f}°')
    
    def get_indices_in_range(self, scan_msg: LaserScan):
        """获取指定角度范围内的索引"""
        # 雷达参数
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        
        # 计算目标范围（弧度）
        target_min_rad = (self.center_angle - self.range_width/2) * np.pi / 180.0
        target_max_rad = (self.center_angle + self.range_width/2) * np.pi / 180.0
        
        # 处理角度环绕（例如 -15° 到 15°）
        indices = []
        for i in range(len(scan_msg.ranges)):
            angle = angle_min + i * angle_increment
            # 将角度转换到 -180 到 180 范围进行比较
            if angle > np.pi:
                angle -= 2 * np.pi
            
            if target_min_rad <= angle <= target_max_rad:
                indices.append(i)
        
        return indices
    
    def scan_callback(self, msg):
        # 获取范围内的索引
        indices = self.get_indices_in_range(msg)
        
        if not indices:
            return
        
        # 获取范围内的有效距离
        distances = []
        for idx in indices:
            dist = msg.ranges[idx]
            if msg.range_min < dist < msg.range_max:
                distances.append((dist, idx))
        
        if not distances:
            return
        
        # 找最小距离
        min_dist, min_idx = min(distances, key=lambda x: x[0])
        
        
        # 精简输出
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # if reliable and min_dist < 1.0:  # 只在有威胁时输出
        #     print(f"[{timestamp:.2f}s] 障碍物: {min_dist:.2f}m | 可靠: ✓ | {reason}")
        # elif not reliable and min_dist < 0.8:
        #     print(f"[{timestamp:.2f}s] 障碍物: {min_dist:.2f}m | 可靠: ✗ | {reason}")

        print(f"[{timestamp:.2f}s] 障碍物: {min_dist:.2f}m")
        

def main(args=None):
    rclpy.init(args=args)
    
    # 默认范围：-15° 到 15°（即中心0°，宽度30°）
    center = 0  # 中心角度
    width = 30.0    # 总宽度
    
    if len(sys.argv) > 1:
        center = float(sys.argv[1])
    if len(sys.argv) > 2:
        width = float(sys.argv[2])
    
    node = SimpleObstacleDetector(center, width)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n检测器已停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()