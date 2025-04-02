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

        # Add a shutdown flag
        self.should_shutdown = False
       
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

        try:
            # Turn on uRAD
            self.turn_on = uRAD_RP_SDK11.turnON()
            
            # Load configuration
            self.config = uRAD_RP_SDK11.loadConfiguration(**self.params)
            if (self.config != 0):
                self.get_logger().error(f'Configuration unsuccessful')
                self.close_program()

        except Exception as e:
            self.get_logger().error(f'Failed to initialize radar: {e}')
            self.close_program()
       

    def publish_radar(self):
            # Check if we should stop
            if self.should_shutdown:
                return
                
            try:
                msg = CwIq()
                return_code, results, raw_results = uRAD_RP_SDK11.detection()
            
                if (return_code != 0):
                    self.get_logger().error(f'Detection error: Data delivery failed! Code: {return_code}')
                    self.close_program()
                    return
                
                msg.i = raw_results[0]
                msg.q = raw_results[1]
            
                self.publisher.publish(msg)
                self.get_logger().info(f'Publishing.. {self.i}')
                self.i += 1
            except Exception as e:
                self.get_logger().error(f'Error during radar detection: {e}')
                self.close_program()


    def close_program(self):
        # First set the shutdown flag to True to ensure the main loop will exit
        self.should_shutdown = True
        # Switch OFF uRAD
        try:
            uRAD_RP_SDK11.turnOFF()
        except Exception as e:
            self.get_logger().error(f'Error turning off radar: {e}')
        finally:
            # node will be shutdown in the main function
            pass
       


def main(args=None):
    rclpy.init(args=args)
    urad_publisher = UradPublisher()
   
    try:
        while rclpy.ok() and not urad_publisher.should_shutdown:
            rclpy.spin_once(urad_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if not urad_publisher.should_shutdown:
            urad_publisher.close_program()
        
        urad_publisher.destroy_node()  # only destroys node
        #rclpy.shutdown()              to shutdown  ROS2 system


if __name__ == '__main__':
    main()