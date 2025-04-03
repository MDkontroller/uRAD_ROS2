import rclpy
from rclpy.node import Node
import urad.uRAD_RP_SDK11 as uRAD_RP_SDK11
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class UradBaseNode(Node):

    """Base class for uRAD nodes with common functionality"""
    
    def __init__(self, node_name):
        super().__init__(node_name)
        # Common initialization
        self.should_shutdown = False
        self.load_params()
        self.initialize_radar()

    def load_params(self):
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
