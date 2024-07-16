import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from autoware_sensing_msgs.msg import GnssInsOrientationStamped
from std_msgs.msg import Header
import serial
import threading
import time

class GNSSDriverNode(Node):

    def __init__(self):
        super().__init__('gnss_driver_node')
        
        self.declare_parameter('port', '/dev/gnss')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'gnss_link')  # 声明frame_id参数
 
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.navsatfix_publisher = self.create_publisher(NavSatFix, 'panwise/nav_sat_fix', 10)
        self.orientation_publisher = self.create_publisher(GnssInsOrientationStamped, 'panwise/autoware_orientation', 10)

        
        self.serial_port = None

        self.connect_serial_port()

        self.serial_thread = threading.Thread(target=self.read_serial_data)
        self.serial_thread.daemon = True
        self.serial_thread.start()

    def connect_serial_port(self):
        while self.serial_port is None:
            try:
                self.serial_port = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=1
                )
                self.get_logger().info(f'Successfully connected to {self.port}')
            except serial.SerialException as e:
                self.get_logger().error(f'Error connecting to serial port: {e}')
                time.sleep(5)  # 等待5秒后重试

    def read_serial_data(self):
        while rclpy.ok():
            if self.serial_port and self.serial_port.is_open:
                try:
                    line = self.serial_port.readline().decode('ascii', errors='replace').strip()
                    if line.startswith('$GPFPD'):
                        self.get_logger().info(f'Received: {line}')
                        data = self.parse_gnss_data(line)
                        if data:
                            msg = NavSatFix()
                            msg.header = Header()
                            msg.header.stamp = self.get_clock().now().to_msg()
                            msg.header.frame_id = self.frame_id  # 使用参数中的frame_id
                            msg.latitude = data['latitude']
                            msg.longitude = data['longitude']
                            msg.altitude = data['altitude']
                            # 其他数据可根据需要发布
                            self.navsatfix_publisher.publish(msg)
                            # self.get_logger().info(f'Publishing: {msg}')
                            orientation_msg = GnssInsOrientationStamped()
                            orientation_msg.header.stamp = self.get_clock().now().to_msg()
                            # orientation_msg.header.frame_id = self.frame_id
                            orientation_msg.orientation.rmse_rotation_z = data['heading']
                            orientation_msg.orientation.rmse_rotation_y = data['pitch']
                            orientation_msg.orientation.rmse_rotation_x = data['roll']
                            self.orientation_publisher.publish(orientation_msg)

                except serial.SerialException as e:
                    self.get_logger().error(f'Serial port error: {e}')
                    self.serial_port.close()
                    self.serial_port = None
                    self.connect_serial_port()
            else:
                self.connect_serial_port()

    def parse_gnss_data(self, line):
        parts = line.split(',')
        if len(parts) < 15:
            self.get_logger().error(f'Invalid data length: {line}')
            return None
        
        try:
            data = {
                'GPSWeek': int(parts[1]),
                'GPSTime': float(parts[2]),
                'heading': float(parts[3]),
                'pitch': float(parts[4]),
                'roll': float(parts[5]),
                'latitude': float(parts[6]),
                'longitude': float(parts[7]),
                'altitude': float(parts[8]),
                'Ve': float(parts[9]),
                'Vn': float(parts[10]),
                'Vu': float(parts[11]),
                'baseline': float(parts[12]),
                'NSV1': int(parts[13]),
                'NSV2': int(parts[14]),
                'status': parts[15].split('*')[0]  # 去掉校验和部分
            }
            return data
        except ValueError as e:
            self.get_logger().error(f'Error parsing data: {e}')
            return None

    def __del__(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

def main(args=None):
    rclpy.init(args=args)
    node = GNSSDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
