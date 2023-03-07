import rclpy
from rclpy.node import Node
#from std_msgs.msg import String
from interfaces.msg import CwIq


class Publisher(Node):

    def __init__(self):
        super().__init__('cw_node_iq')
        #self.publisher = self.create_publisher(String, 'string_topic', 5)
        self.publisher = self.create_publisher(CwIq, 'cw_iq', 3)
        repetition = 0.5
        self.timer = self.create_timer(repetition, self.timer_callback)
        self.i     = 0

    def timer_callback(self):

        #msg      = String()
        msg      = CwIq()
        #msg.data = 'ROS2 counter: %d' % self.i
        msg.i    = [4096, 4095, self.i]
        msg.q    = [ self.i]
        #self.publisher.publish(msg)
        self.publisher.publish(msg)
        self.get_logger().info('publishing: "%s"' % str(msg.i))
        self.i   += 1

def main(args=None):

    rclpy.init(args=args)
    publisher_s = Publisher()
    rclpy.spin(publisher_s)

if __name__ == '__main__':
    main()



