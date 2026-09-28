#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading
import sys
import time

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        
        # Parameters
        self.declare_parameter('port', '/dev/arduino')
        self.declare_parameter('baudrate', 115200)
        
        port_name = self.get_parameter('port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        # Serial Setup
        try:
            self.ser = serial.Serial(port_name, baud_rate, timeout=1.0)
            self.get_logger().info(f"Connected to {port_name} at {baud_rate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            sys.exit(1)

        # Publishers and Subscribers
        self.pub_serial_out = self.create_publisher(String, '/serial_out', 10)
        self.sub_serial_in = self.create_subscription(
            String,
            '/serial_in',
            self.serial_in_callback,
            10
        )

        self.port_name = port_name
        self.baud_rate = baud_rate
        self.ser_lock = threading.Lock()


        # Start a daemon thread to read from the serial port
        self.read_thread = threading.Thread(target=self.read_serial_loop)
        self.read_thread.daemon = True
        self.read_thread.start()

    def serial_in_callback(self, msg):
        """Callback for incoming ROS messages. Writes data to Arduino."""
        data = msg.data
        if not data.endswith('\n'):
            data += '\n'

        with self.ser_lock:
            try:
                if self.ser and self.ser.is_open:
                    self.ser.write(data.encode('utf-8'))
                    self.get_logger().debug(f"Wrote to serial: {data.strip()}")
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f"Failed to write to serial: {e}")
    
    def _reconnect_serial(self):
        while rclpy.ok():
            with self.ser_lock:
                try:
                    if self.ser and self.ser.is_open:
                        self.ser.close()
                except Exception:
                    pass
                try:
                    self.ser = serial.Serial(self.port_name, self.baud_rate, timeout=1.0)
                    self.get_logger().info(f"Reconnected to {self.port_name} at {self.baud_rate} baud.")
                    return True
                except (serial.SerialException, OSError) as e:
                    self.get_logger().warn(f"Reconnect failed for {self.port_name}: {e}")
            time.sleep(1.0)
        return False

    def read_serial_loop(self):
        """Background thread loop for reading Arduino telemetry."""
        while rclpy.ok():
            try:
                with self.ser_lock:
                    if not self.ser or not self.ser.is_open:
                        raise serial.SerialException("Serial port is not open")
                    waiting = self.ser.in_waiting
                    line = ""
                    if waiting > 0:
                        line = self.ser.readline().decode('utf-8', errors='replace').strip()
                if line:
                    msg = String()
                    msg.data = line
                    self.pub_serial_out.publish(msg)
                else:
                    time.sleep(0.01)
            except (serial.SerialException, OSError) as e:
                if not rclpy.ok():
                    break
                self.get_logger().error(f"Serial I/O error in read thread: {e}. Reconnecting...")
                if not self._reconnect_serial():
                    break
            except Exception as e:
                if not rclpy.ok():
                    break
                self.get_logger().error(f"Unexpected error in read thread: {e}")
                time.sleep(0.2)

    def destroy_node(self):
        with self.ser_lock:
            try:
                if self.ser and self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    bridge_node = SerialBridge()
    
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        bridge_node.get_logger().info("Shutting down serial bridge node.")
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
