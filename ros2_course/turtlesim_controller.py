import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class TurtlesimController(Node):
        def __init__(self):
            super().__init__('turtlesim_controller')
            self.twist_pub = self.create_publisher(
                            Twist, '/turtle1/cmd_vel', 10
            )
            self.pose = None
            self.subscription = self.create_subscription(
               Pose,
               '/turtle1/pose',
                self.update_pose,
                10
            )

        def update_pose(self, msg):
             self.pose = msg
             self.pose.x = round(self.pose.x, 4)
             self.pose.y = round(self.pose.y, 4)
        
        def move_forward(self, distance=4, speed=1.0):
            vel_msg = Twist()
            if distance > 0:
                vel_msg.linear.x = speed
            else:
                vel_msg.linear.x = -speed

            self.twist_pub.publish(vel_msg) #publish msg

            time_to_stop = self.get_clock().now() + rclpy.time.Duration(seconds=abs(distance / speed))

            # Publish msg while the calculated time is up
            while (self.get_clock().now() <= time_to_stop) and rclpy.ok():
                self.twist_pub.publish(vel_msg)
                rclpy.spin_once(self)   # enélkül errorunk van!!!
            
            # we arrived, set velocity to 0 and publish msg
            vel_msg.linear.x = 0.0
            self.twist_pub.publish(vel_msg)
            self.get_logger().info(f'Turtle Pose: x={self.pose.x}, y={self.pose.y}, theta={math.degrees(self.pose.theta)}')

        def get_adjusted_angular_vel(self, angle_error, angular_speed):
            log_comp = math.log((angle_error / 10) + (1 / 20) + 1)
            sin_comp = math.sin(100 * angle_error) * 0.0005
            return (angular_speed * (log_comp + sin_comp))

        def turn_turtle(self, target_angle, angular_speed):
            while self.pose is None and rclpy.ok():
                self.get_logger().info('Waiting for pose...')
                rclpy.spin_once(self)

            goal_angle = math.degrees(self.pose.theta) + target_angle
                        
            while abs(goal_angle - math.degrees(self.pose.theta)) > 0.05 and rclpy.ok():
                angle_error = goal_angle - math.degrees(self.pose.theta)
                self.get_logger().info(f'angle_error: {angle_error}')

                #regulate between -180 & 180
                if(goal_angle<-180):
                    goal_angle+=360
                if (goal_angle>180.0):
                    goal_angle-=360

                if(angle_error>180):
                    angle_error=angle_error-180
                if(angle_error>0.0):
                    angular_velocity = self.get_adjusted_angular_vel(angle_error, angular_speed)
                if(angle_error<0.0):
                    angle_error=abs(angle_error)
                    angular_velocity = -1*self.get_adjusted_angular_vel(angle_error, angular_speed)
                            

                vel_msg = Twist()
                vel_msg.angular.z=angular_velocity/2
                self.twist_pub.publish(vel_msg)
                rclpy.spin_once(self)

            # Stop the rotation & publish
            vel_msg = Twist()
            vel_msg.angular.z = 0.0
            self.twist_pub.publish(vel_msg)
            rclpy.spin_once(self)
            #self.get_logger().info(f'Turtle Pose: x={self.pose.x}, y={self.pose.y}, theta={math.degrees(self.pose.theta)}')


        def draw_koch_curve(self, level, size, angular_speed):
            if level == 0:
                self.move_forward(size)
            else:
                self.draw_koch_curve(level - 1, size / 3,angular_speed)
                self.turn_turtle(60,angular_speed)
                self.draw_koch_curve(level - 1, size / 3,angular_speed)
                self.turn_turtle(-120,angular_speed)
                self.draw_koch_curve(level - 1, size / 3,angular_speed)
                self.turn_turtle(60,angular_speed)
                self.draw_koch_curve(level - 1, size / 3,angular_speed)

        def main_loop(self, level=2, size=4, angular_speed=1.0):
            self.move_forward(size/3)
            for _ in range(3):
                self.draw_koch_curve(level, size,angular_speed)
                self.turn_turtle(-120.0,angular_speed)


def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtlesimController()

    turtle_controller.main_loop()
    '''turtle_controller.main_loop(
        level=4,
        size=4,
        angular_speed=1.0
    )'''

    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
