#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import sys
import numpy as np

class ScanMonitor(Node):
    def __init__(self, target_angle_deg=0.0, window_width_deg=45.0, adaptive_window=True):
        super().__init__('scan_monitor')
        
        # 目标角度（度），范围 0-360
        self.target_angle_deg = self.normalize_angle_360(target_angle_deg)
        # 期望窗口宽度（度），默认±45度
        self.desired_window_width_deg = window_width_deg
        # 是否启用自适应窗口（超出边界时自动调整）
        self.adaptive_window = adaptive_window
        
        # 订阅/scan话题
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.get_logger().info(f'ScanMonitor已启动')
        self.get_logger().info(f'  目标角度: {self.target_angle_deg}°')
        self.get_logger().info(f'  期望范围: {self.target_angle_deg - window_width_deg}° ~ {self.target_angle_deg + window_width_deg}°')
        self.get_logger().info(f'  自适应窗口: {"启用" if adaptive_window else "禁用"}')
        self.get_logger().info('订阅话题: /scan')
    
    def normalize_angle_360(self, angle_deg):
        """
        将角度归一化到 0-360 度范围
        """
        angle_deg = angle_deg % 360
        if angle_deg < 0:
            angle_deg += 360
        return angle_deg
    
    def get_window_indices(self, target_angle_deg, scan_msg):
        """
        获取窗口内的所有索引，正确处理跨越0°的情况
        返回: (indices_list, actual_window_info, warning_msg)
        """
        # 归一化目标角度到 0-360
        target_angle_deg = self.normalize_angle_360(target_angle_deg)
        
        # 获取雷达的角度范围（转换为度数，0-360）
        radar_min_deg = scan_msg.angle_min * 180 / np.pi
        radar_max_deg = scan_msg.angle_max * 180 / np.pi
        
        # 将雷达角度转换到 0-360 范围
        if radar_min_deg < 0:
            radar_min_deg += 360
            radar_max_deg += 360
        
        # 计算窗口边界（度数）
        half_width_deg = self.desired_window_width_deg
        start_deg = target_angle_deg - half_width_deg
        end_deg = target_angle_deg + half_width_deg
        
        indices = []
        actual_ranges = []
        warning_msg = None
        
        # 检查是否需要跨越0°边界
        crosses_zero = start_deg < 0 or end_deg > 360
        
        if crosses_zero:
            # 情况1：窗口跨越0°，分成两个区间 [0, end] 和 [start+360, 360]
            if start_deg < 0:
                # 区间1: 0 到 end_deg
                interval1_start = 0
                interval1_end = end_deg
                # 区间2: start_deg + 360 到 360
                interval2_start = start_deg + 360
                interval2_end = 360
            else:  # end_deg > 360
                # 区间1: start_deg 到 360
                interval1_start = start_deg
                interval1_end = 360
                # 区间2: 0 到 end_deg - 360
                interval2_start = 0
                interval2_end = end_deg - 360
            
            # 获取第一个区间的索引
            idx1_start, idx1_end = self.angle_range_to_indices(interval1_start, interval1_end, scan_msg, radar_min_deg, radar_max_deg)
            if idx1_start is not None and idx1_end is not None:
                if idx1_start <= idx1_end:
                    indices.extend(range(idx1_start, idx1_end + 1))
                else:
                    indices.extend(range(idx1_start, len(scan_msg.ranges)))
                    indices.extend(range(0, idx1_end + 1))
                actual_ranges.append(f"{interval1_start:.1f}°-{interval1_end:.1f}°")
            
            # 获取第二个区间的索引
            idx2_start, idx2_end = self.angle_range_to_indices(interval2_start, interval2_end, scan_msg, radar_min_deg, radar_max_deg)
            if idx2_start is not None and idx2_end is not None:
                if idx2_start <= idx2_end:
                    indices.extend(range(idx2_start, idx2_end + 1))
                else:
                    indices.extend(range(idx2_start, len(scan_msg.ranges)))
                    indices.extend(range(0, idx2_end + 1))
                actual_ranges.append(f"{interval2_start:.1f}°-{interval2_end:.1f}°")
            
            warning_msg = f'⚠️ 窗口跨越0°边界，已分割为两个区间: {", ".join(actual_ranges)}'
            
        else:
            # 情况2：窗口不跨越0°，直接处理
            # 检查是否超出雷达范围
            if start_deg < radar_min_deg or end_deg > radar_max_deg:
                if self.adaptive_window:
                    # 裁剪到雷达范围
                    start_deg = max(start_deg, radar_min_deg)
                    end_deg = min(end_deg, radar_max_deg)
                    warning_msg = f'⚠️ 窗口超出雷达范围，已裁剪为: {start_deg:.1f}°-{end_deg:.1f}°'
                else:
                    self.get_logger().error(f'窗口超出雷达范围且自适应窗口已禁用')
                    return None, None, None
            
            # 获取索引
            idx_start, idx_end = self.angle_range_to_indices(start_deg, end_deg, scan_msg, radar_min_deg, radar_max_deg)
            if idx_start is not None and idx_end is not None:
                if idx_start <= idx_end:
                    indices = list(range(idx_start, idx_end + 1))
                else:
                    indices = list(range(idx_start, len(scan_msg.ranges)))
                    indices.extend(range(0, idx_end + 1))
        
        # 去重并排序
        indices = sorted(set(indices))
        
        # 计算实际覆盖的角度范围
        actual_width = len(indices) * (scan_msg.angle_increment * 180 / np.pi) if indices else 0
        
        return indices, actual_width, warning_msg
    
    def angle_range_to_indices(self, start_deg, end_deg, scan_msg, radar_min_deg, radar_max_deg):
        """
        将角度范围转换为索引范围
        """
        # 裁剪到雷达范围
        start_deg = max(start_deg, radar_min_deg)
        end_deg = min(end_deg, radar_max_deg)
        
        if start_deg >= end_deg:
            return None, None
        
        # 转换回雷达原始坐标系（可能是负角度）
        radar_min_rad = scan_msg.angle_min
        angle_increment_rad = scan_msg.angle_increment
        
        # 将角度转换到雷达坐标系
        start_rad = start_deg * np.pi / 180
        end_rad = end_deg * np.pi / 180
        
        # 如果雷达角度范围是负的，需要调整
        if scan_msg.angle_min < 0:
            # 将0-360的角度转换到 -180 到 180
            if start_rad > np.pi:
                start_rad -= 2 * np.pi
            if end_rad > np.pi:
                end_rad -= 2 * np.pi
        
        # 计算索引
        start_idx = int((start_rad - scan_msg.angle_min) / angle_increment_rad)
        end_idx = int((end_rad - scan_msg.angle_min) / angle_increment_rad)
        
        # 边界检查
        start_idx = max(0, min(start_idx, len(scan_msg.ranges) - 1))
        end_idx = max(0, min(end_idx, len(scan_msg.ranges) - 1))
        
        return start_idx, end_idx
    
    def calculate_window_statistics(self, ranges, intensities, indices):
        """
        计算窗口内数据的统计信息
        """
        if not indices:
            return None
            
        # 提取窗口内的距离和强度数据
        window_ranges = []
        window_intensities = []
        invalid_count = 0
        
        for idx in indices:
            if idx < len(ranges):
                r = ranges[idx]
                # 过滤无效数据
                if r == float('inf'):
                    invalid_count += 1
                elif r == float('-inf'):
                    invalid_count += 1
                elif r == 0:
                    invalid_count += 1
                else:
                    window_ranges.append(r)
                    
                if intensities and idx < len(intensities):
                    i = intensities[idx]
                    if i is not None and i > 0:
                        window_intensities.append(i)
        
        if not window_ranges:
            return None
            
        # 计算统计信息
        stats = {
            'num_points': len(indices),
            'num_valid_ranges': len(window_ranges),
            'num_invalid': invalid_count,
            'num_valid_intensities': len(window_intensities),
            'invalid_ratio': invalid_count / len(indices) if indices else 0,
            
            # 距离统计
            'range_min': np.min(window_ranges),
            'range_max': np.max(window_ranges),
            'range_mean': np.mean(window_ranges),
            'range_median': np.median(window_ranges),
            'range_std': np.std(window_ranges),
        }
        
        # 添加分位数
        stats['range_percentile_25'] = np.percentile(window_ranges, 25)
        stats['range_percentile_75'] = np.percentile(window_ranges, 75)
        
        # 计算最佳估计值
        stats['best_estimate'] = stats['range_median']
        
        # 计算置信区间
        stats['confidence_lower'] = stats['range_mean'] - stats['range_std']
        stats['confidence_upper'] = stats['range_mean'] + stats['range_std']
        
        # IQR鲁棒估计
        iqr = stats['range_percentile_75'] - stats['range_percentile_25']
        stats['robust_lower'] = stats['range_median'] - 1.5 * iqr
        stats['robust_upper'] = stats['range_median'] + 1.5 * iqr
        
        return stats
    
    def scan_callback(self, msg):
        # 获取窗口内的索引
        result = self.get_window_indices(self.target_angle_deg, msg)
        
        if result[0] is None:
            return
        
        indices, actual_width, warning_msg = result
        
        if not indices:
            return
        
        # 打印警告信息（如果有）
        if warning_msg:
            self.get_logger().warn(warning_msg)
        
        # 计算窗口统计信息
        stats = self.calculate_window_statistics(msg.ranges, msg.intensities, indices)
        
        if stats is None:
            self.get_logger().warn('窗口内无有效数据')
            return
        
        # 打印信息
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        print(f"\n{'='*70}")
        print(f"[时间: {timestamp:.3f}s]")
        print(f"{'='*70}")
        
        print(f"📊 融合范围信息:")
        print(f"  目标角度: {self.target_angle_deg}°")
        print(f"  期望范围: ±{self.desired_window_width_deg}°")
        print(f"  实际宽度: {actual_width:.1f}°")
        print(f"  总点数: {stats['num_points']}")
        print(f"  有效距离点数: {stats['num_valid_ranges']}/{stats['num_points']}")
        print(f"  无效数据点数: {stats['num_invalid']} ({stats['invalid_ratio']*100:.1f}%)")
        
        print(f"\n📏 距离统计 (单位: 米):")
        print(f"  最小值: {stats['range_min']:.3f}m")
        print(f"  最大值: {stats['range_max']:.3f}m")
        print(f"  平均值: {stats['range_mean']:.3f}m")
        print(f"  中位数: {stats['range_median']:.3f}m")
        print(f"  标准差: {stats['range_std']:.3f}m")
        print(f"  25%分位: {stats['range_percentile_25']:.3f}m")
        print(f"  75%分位: {stats['range_percentile_75']:.3f}m")
        
        if stats['num_valid_intensities'] > 0:
            print(f"\n💡 强度统计 (平均):")
            avg_intensity = np.mean([msg.intensities[i] for i in indices 
                                    if i < len(msg.intensities) 
                                    and msg.intensities[i] > 0])
            print(f"  平均强度: {avg_intensity:.2f}")
        
        print(f"\n✨ 融合结果:")
        print(f"  ★ 最佳估计距离 (中位数): {stats['best_estimate']:.3f}m")
        print(f"  均值置信区间: [{stats['confidence_lower']:.3f}m, {stats['confidence_upper']:.3f}m]")
        print(f"  鲁棒估计区间: [{stats['robust_lower']:.3f}m, {stats['robust_upper']:.3f}m]")
        
        # 数据质量评估
        cv = stats['range_std'] / stats['range_mean'] if stats['range_mean'] > 0 else 1.0
        print(f"\n📈 数据质量:")
        if stats['invalid_ratio'] > 0.3:
            print(f"  ⚠️  无效数据比例过高: {stats['invalid_ratio']*100:.1f}%")
        if cv < 0.05:
            print(f"  ✓ 变异系数: {cv:.3f} (优秀)")
        elif cv < 0.1:
            print(f"  ✓ 变异系数: {cv:.3f} (良好)")
        elif cv < 0.2:
            print(f"  - 变异系数: {cv:.3f} (一般)")
        else:
            print(f"  ⚠️  变异系数: {cv:.3f} (较差)")
        
        print(f"{'='*70}\n")

def main(args=None):
    rclpy.init(args=args)
    
    # 从命令行参数获取配置
    target_angle = 0.0
    window_width = 15.0
    adaptive = True
    
    if len(sys.argv) > 1:
        try:
            target_angle = float(sys.argv[1])
            print(f"命令行参数: 目标角度 {target_angle}°")
        except ValueError:
            print(f"无效的角度参数: {sys.argv[1]}, 使用默认值 0°")
    
    if len(sys.argv) > 2:
        try:
            window_width = float(sys.argv[2])
            print(f"命令行参数: 窗口宽度 ±{window_width}°")
        except ValueError:
            print(f"无效的宽度参数: {sys.argv[2]}, 使用默认值 ±45°")
    
    if len(sys.argv) > 3:
        adaptive = sys.argv[3].lower() in ['true', '1', 'yes', 'on']
        print(f"命令行参数: 自适应模式 {adaptive}")
    
    node = ScanMonitor(target_angle, window_width, adaptive)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n节点已停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()