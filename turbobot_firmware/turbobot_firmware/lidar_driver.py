#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import math
import time

class LidarDataPublisher(Node):
    def __init__(self):
        super().__init__('lidar_data_publisher')
        
        self.DATA_LENGTH = 7  # Data length: angle, speed, distance 1-4, checksum
        self.MAX_DISTANCE = 3.0  # Maximum distance in meters
        self.MIN_DISTANCE = 0.1  # Minimum distance in meters
        self.port = '/dev/pts/11'  # Specify serial port
        self.BAUDRATE = 115200
        self.MAX_DATA_SIZE = 360  # Number of data points (1 degree resolution)
        self.COUNT = 0
        
        self.laser_scan_publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.serial_connection = self.connect_serial(self.port, self.BAUDRATE)
        self.data = {
            'angles': [],
            'distances': []
        }
        
    def connect_serial(self, port: str, baudrate: int):
        try:
            ser = serial.Serial(port, baudrate, timeout=1)
            ser.reset_input_buffer()
            self.get_logger().info(f"Serial connection established at {port}")
            return ser
        except Exception as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            return None

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = 0.0
        scan.angle_max = 2 * math.pi
        scan.angle_increment = math.pi / 180.0  # 1 degree in radians
        scan.time_increment = 0.0  # Assuming a stationary scanner
        scan.scan_time = 0.1  # Approximate scan time
        scan.range_min = self.MIN_DISTANCE
        scan.range_max = self.MAX_DISTANCE
        
        ranges = [float('inf')] * self.MAX_DATA_SIZE  # Initialize ranges with 'inf'

        for angle, distance in zip(self.data['angles'], self.data['distances']):
            index = int(angle * 180 / math.pi) % self.MAX_DATA_SIZE  # Map angle to index
            ranges[index] = distance
        
        scan.ranges = ranges
        scan.intensities = []  # No intensity data
        
        self.laser_scan_publisher.publish(scan)
        self.get_logger().info("LaserScan message published")
        
        # Clear data for the next scan
        self.data['angles'].clear()
        self.data['distances'].clear()

    def read_lidar_data(self):
        if not self.serial_connection:
            self.get_logger().error("Serial connection not established")
            return
        
        while rclpy.ok():
            try:
                if self.serial_connection.in_waiting > 0:
                    line = self.serial_connection.readline().decode().strip()
                    sensor_data = line.split('\t')
                    
                    if len(sensor_data) == self.DATA_LENGTH:
                        for i in range(2, 6):  # Process 4 distance readings
                            try:
                                angle = (int(sensor_data[0]) + i - 1) * math.pi / 180.0  # Radians
                                distance = float(sensor_data[i]) / 1000.0  # Convert mm to meters
                                
                                if self.MIN_DISTANCE <= distance <= self.MAX_DISTANCE:
                                    self.data['angles'].append(angle)
                                    self.data['distances'].append(distance)
                                    self.get_logger().info(f"angle : {angle}")
                                    self.get_logger().info(f"distance : {distance}")
                                    self.get_logger().info(f"count : {self.COUNT}")
                                    self.get_logger().info("------------------------")
                                    
                            except ValueError:
                                print("Value error")
                                continue

                        if len(self.data['angles']) >= 350:
                            self.publish_scan()
                            self.COUNT += 1
            except Exception as e:
                self.get_logger().error(f"Error reading LiDAR data: {e}")
            time.sleep(0.001)  # Slight delay to reduce CPU usage

def main(args=None):
    rclpy.init(args=args)
    node = LidarDataPublisher()
    try:
        node.read_lidar_data()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
