import rclpy
from rclpy.node import Node
from interfaces.msg import CwIq

class RadarSubscriber(Node):

    def __init__(self):
        super().__init__('sub_cw_node')
        #self.subscriction = self.create_subscription(String, 'string_topic', self.listener_callback, 5)
        self.subscription = self.create_subscription(CwIq, 'cw_iq', self.listener_callback,5 )
        self.subscription # prevent unused variable warning
        self.get_logger().info('subcribSer set to hear ... ')

    def listener_callback(self, msg):
        
        """Process incoming radar data"""
        self.get_logger().info('hearing counter:  "%s" ' % str(msg.q))



def main(args=None):
    rclpy.init(args=args)
    subscriber = RadarSubscriber()
    
    try:
        rclpy.spin(subscriber)

    except Exception as e:
        subscriber.get_logger().error(f'Errror in subcriber Node!')
        pass
    finally:

        subscriber.destroy_node()  # just destroy subscriber node
        #rclpy.shutdown()


if __name__ == '__main__':
    main()