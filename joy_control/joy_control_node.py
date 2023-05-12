# write ros 2 node that subscribes to joy topic and publishes to cmd_vel topic /joy topic is a sensor_msgs/Joy message type
# publish to /commands/motor/current: std_msgs/msg/Float64
# publish to /commands/servo/position: std_msgs/msg/Float64
# /home/nuc/f1tenth_ws/src/joy_control/joy_control/joy_control_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

class JoyControl(Node):
    def __init__(self):
        super().__init__('joy_control')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            1)
        
        self.subscription  # prevent unused variable warning
        self.current_publisher = self.create_publisher(Float64, '/commands/motor/current', 10)
        # used for braking
        self.speed_publisher = self.create_publisher(Float64, '/commands/motor/speed', 10)
        self.angle_publisher = self.create_publisher(Float64, '/commands/servo/position', 10)
        self.braking_publisher = self.create_publisher(Float64, '/commands/motor/brake', 10)

        self.current_input_list = []

        self.zero_current_threshold_ = 0.1
        self.current = Float64()
        self.angle = Float64()
        self.braking_speed  = Float64()
        self.braking_speed.data = 0.0

        self.breaking = Float64()


    def convert_to_servo_angle(self, joy_input):
        # servo angle is between 0.15 and 0.85
        # angle is between -1 and 1
        # 1 -> 0.15
        # 0 -> 0.5
        # -1 -> 0.85
        return 0.5 - 0.35 * joy_input

    def convert_to_motor_current(self, joy_input):
        # curent is between -1 and 1
        # 1 -> 0.0
        # -1 -> 7.5
 
        
        return 4 * (1 - joy_input)
    
    def braking_enabled(self, joy_input):
        return joy_input < 0.0
    
    def repetetive_imput(self):
        # if all the inputs are the same then return true
        return len(set(self.current_input_list)) == 1


    def listener_callback(self, msg):
        # on cable curent is on msg 5 and breaking is msg 2
        self.angle.data = self.convert_to_servo_angle(msg.axes[0])
        self.current.data = self.convert_to_motor_current(msg.axes[5])
        self.get_logger().info('Input joy speed: "%s"' % msg.axes[5])

        self.current_input_list.append(self.current.data)
        
        if len(self.current_input_list) > 4:
            self.current_input_list.pop(0)

        self.angle_publisher.publish(self.angle)
        if self.braking_enabled(msg.axes[2]):   

            self.breaking.data = 2000.0
            self.braking_publisher.publish(self.breaking)
            self.get_logger().info('breaking')
        else:
            self.breaking.data = 0.0
            # self.braking_publisher.publish(self.breaking)
            self.get_logger().info('Current: "%s"' % self.current.data)
            self.current_publisher.publish(self.current)


def main(args=None):
    rclpy.init(args=args)
    joy_control = JoyControl()
    rclpy.spin(joy_control)
    joy_control.destroy_node()
    rclpy.shutdown()
