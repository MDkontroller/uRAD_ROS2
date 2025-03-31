import rclpy
import urad.uRAD_RP_SDK11 as uRAD_RP_SDK11
from rclpy.node import Node
from interfaces.msg import CwIq
import yaml
import os

class UradPublisher(Node):
    
    def __init__(self):
        super().__init__('urad_publisher')
        
        # Create publisher
        self.publisher = self.create_publisher(CwIq, 'cw_iq', 10)
        
        # Load parameters from YAML
        self.load_params_from_yaml()
        
        # Initialize radar
        self.initialize_radar()
        
        # Create timer for publishing
        self.timer = self.create_timer(0.1, self.publish_radar)  # 10Hz
        self.i = 0
        
    def load_params_from_yaml(self):
        # Path to your YAML file
        yaml_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'urad_config.yaml')
        
        try:

            with open(yaml_path, 'r') as file:
                self.params = yaml.safe_load(file)
                self.get_logger().info(f'Loaded parameters from {yaml_path}')

        except Exception as e:

            self.get_logger().error(f'Failed to load parameters from YAML: {e}')
            self.params = {
                'f0': 125, 'BW': 0, 'Ns': 200, 'I_true': True, 'Q_true': True,
                'Ntar': 4, 'MTI': 0, 'Mth': 0, 'Rmax': 100, 'Alpha': 5,
                'distance_true': False, 'velocity_true': True, 'SNR_true': True,
                'movement_true': False, 'mode': 1
            }
            self.get_logger().info('Using default parameters')
        
    def initialize_radar(self):
        # Turn on uRAD
        self.turn_on = uRAD_RP_SDK11.turnON()
        
        # Load configuration
        self.config = uRAD_RP_SDK11.loadConfiguration(**self.params)
        if (self.config != 0):
            self.get_logger().error('uRAD_RP_SDK11.loadConfiguration: config unsuccessful')
            self.close_program()
            
    def publish_radar(self):
        msg = CwIq()
        return_code, results, raw_results = uRAD_RP_SDK11.detection()
        
        if (return_code != 0):
            self.get_logger().error('Detection error: Data delivery failed!')
            self.close_program()
            
        msg.i = raw_results[0]
        msg.q = raw_results[1]
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing.. {self.i}')
        self.i += 1
        
    def close_program(self):
        # Switch OFF uRAD
        uRAD_RP_SDK11.turnOFF()
        exit()

def main(args=None):
    rclpy.init(args=args)
    urad_publisher = UradPublisher()
    
    try:
        rclpy.spin(urad_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        urad_publisher.close_program()
        rclpy.shutdown()

if __name__ == '__main__':
    main()





import rclpy
import urad.uRAD_RP_SDK11 as uRAD_RP_SDK11
from rclpy.node import Node
from interfaces.msg import CwIq
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class UradPublisher(Node):
    def __init__(self):
        super().__init__('urad_publisher')
       
        # Create publisher
        self.publisher = self.create_publisher(CwIq, 'cw_iq', 10)
       
        # Load parameters from YAML
        self.load_params_from_yaml()
       
        # Initialize radar
        self.initialize_radar()
       
        # Create timer for publishing
        self.timer = self.create_timer(0.1, self.publish_radar)  # 10Hz
        self.i = 0
       
    def load_params_from_yaml(self):
        # Get path to config file using ROS 2 package system
        yaml_path = os.path.join(
            get_package_share_directory('urad'),
            'config',
            'urad_config.yaml'
        )
       
        try:
            with open(yaml_path, 'r') as file:
                self.params = yaml.safe_load(file)
                self.get_logger().info(f'Loaded parameters from {yaml_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load parameters from YAML: {e}')
            self.params = {
                'f0': 125, 'BW': 0, 'Ns': 200, 'I_true': True, 'Q_true': True,
                'Ntar': 4, 'MTI': 0, 'Mth': 0, 'Rmax': 100, 'Alpha': 5,
                'distance_true': False, 'velocity_true': True, 'SNR_true': True,
                'movement_true': False, 'mode': 1
            }
            self.get_logger().info('Using default parameters')
       
    def initialize_radar(self):
        # Turn on uRAD
        self.turn_on = uRAD_RP_SDK11.turnON()
       
        # Load configuration
        self.config = uRAD_RP_SDK11.loadConfiguration(**self.params)
        if (self.config != 0):
            self.get_logger().error('uRAD_RP_SDK11.loadConfiguration: config unsuccessful')
            self.close_program()
           
    def publish_radar(self):
        msg = CwIq()
        return_code, results, raw_results = uRAD_RP_SDK11.detection()
       
        if (return_code != 0):
            self.get_logger().error('Detection error: Data delivery failed!')
            self.close_program()
           
        msg.i = raw_results[0]
        msg.q = raw_results[1]
       
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing.. {self.i}')
        self.i += 1
       
    def close_program(self):
        # Switch OFF uRAD
        uRAD_RP_SDK11.turnOFF()
        exit()

def main(args=None):
    rclpy.init(args=args)
    urad_publisher = UradPublisher()
   
    try:
        rclpy.spin(urad_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        urad_publisher.close_program()
        rclpy.shutdown()

if __name__ == '__main__':
    main()