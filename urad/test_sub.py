import rclpy
from rclpy.node import Node
from interfaces.msg import CwIq
#from std_msgs.msg import String

class Processing(Node):

    def __init__(self):
        super().__init__('processing_cw_node')
        #self.subscriction = self.create_subscription(String, 'string_topic', self.listener_callback, 5)
        self.subscribtion = self.create_subscription(CwIq, 'cw_iq', self.listener_callback, 5 )
        self.subscribtion   # prevent unused variable warning
        self.get_logger().info('subcriber set to hear ... ')

    #def timer_callback()

    def listener_callback(self, msg):

        self.get_logger().info('hearing counter:  "%s" ' % str(msg.q))

def main(args=None):

    rclpy.init(args=args)
    subscriber = Processing()
    rclpy.spin(subscriber)


if __name__ == '__main__':
    main()