# write ros 2 node that subscribes to joy topic and publishes to cmd_vel topic /joy topic is a sensor_msgs/Joy message type
# publish to /commands/motor/current: std_msgs/msg/Float64
# publish to /commands/servo/position: std_msgs/msg/Float64
# /home/nuc/f1tenth_ws/src/joy_control/joy_control/joy_control_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
import numpy as np 

class JoyControl(Node):
    def __init__(self):
        super().__init__('joy_control')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            1)
        
        self.joy_incorrect_mode = False
        self.watchdog_activated = False
        self.watchdog_timer = self.create_timer(0.2, self.watchdog_callback)
        
        self.subscription  # prevent unused variable warning
        self.current_publisher = self.create_publisher(Float64, '/commands/motor/current', 10)
        # used for braking
        self.speed_publisher = self.create_publisher(Float64, '/commands/motor/speed', 10)
        self.angle_publisher = self.create_publisher(Float64, '/commands/servo/position', 10)
        self.braking_publisher = self.create_publisher(Float64, '/commands/motor/brake', 10)

        self.zero_current_threshold_ = 0.1
        self.current = Float64()
        self.angle = Float64()
        self.braking_speed  = Float64()
        self.braking_speed.data = 0.0
        self.breaking = Float64()
        self.MAX_CURRENT = 8.0
        self.joy_last_update = self.get_clock().now()
        
        
    def watchdog_callback(self):
        t_now_s = self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1] * 10e-9
        t_update_s = self.joy_last_update.seconds_nanoseconds()[0] + self.joy_last_update.seconds_nanoseconds()[1] * 10e-9
        if t_now_s - t_update_s  > 0.1:
            self.watchdog_activated = True
            #self.get_logger().info(f'watchdog activated')
            self.breaking.data = 2000.0
            self.braking_publisher.publish(self.breaking)
            return


    def convert_to_servo_angle(self, joy_input):
        # servo angle is between 0.15 and 0.85
        # angle is between -1 and 1
        # 1 -> 0.15
        # 0 -> 0.5
        # -1 -> 0.85
        return 0.5 - 0.35 * joy_input

    def convert_to_motor_current(self, joy_input):
        # 0 -> 0.0
        # 1 -> MAX_CURRENT       
        return self.MAX_CURRENT * joy_input
    
    
    def braking_enabled(self, joy_input):
        return joy_input > 0.5
    
    
    def check_joy_msg_format(self, msg):
        if len(msg.axes) != 8:
            self.get_logger().info('Joy is in incorrect mode.')
            self.joy_incorrect_mode = True
           
        if(np.isclose(msg.axes[6], 1.0) or np.isclose(msg.axes[7], 1.0)) or (np.isclose(msg.axes[6], -1.0) or np.isclose(msg.axes[7], -1.0)):
            self.get_logger().info('Joy is in incorrect mode.')
            self.joy_incorrect_mode = True
            


    def joy_callback(self, msg):
        self.check_joy_msg_format(msg)

        self.joy_last_update = self.get_clock().now() 
        
        self.get_logger().info(f'self.joy_last_update: {self.joy_last_update}')
        
        joy_braking_input =  (- msg.axes[2] + 1.0) / 2.0 # 0.0 to 1.0
        
        joy_acceleration_input =  (- msg.axes[5] + 1.0) / 2.0 # 0.0 to 1.0
        
        steering_input = msg.axes[0]
        
        self.angle.data = self.convert_to_servo_angle(steering_input)
        self.angle_publisher.publish(self.angle)
        
        self.current.data = self.convert_to_motor_current(joy_acceleration_input)
        
        if msg.buttons[0] and msg.buttons[1] and msg.buttons[2] and msg.buttons[3]:
            self.watchdog_activated = False
            self.joy_incorrect_mode = False
            self.get_logger().info('resetting errors')  
        
        if self.braking_enabled(joy_braking_input) or self.watchdog_activated or self.joy_incorrect_mode:   
            self.breaking.data = 2000.0
            self.braking_publisher.publish(self.breaking)
            self.get_logger().info(f'breaking {self.braking_enabled(joy_braking_input)} {self.watchdog_activated} {self.joy_incorrect_mode}')
        else:
            self.breaking.data = 0.0
            self.get_logger().info(f'current: {self.watchdog_activated} {self.current.data}')
            #self.current_publisher.publish(self.current)



def main(args=None):
    rclpy.init(args=args)
    joy_control = JoyControl()
    rclpy.spin(joy_control)
    joy_control.destroy_node()
    rclpy.shutdown()
