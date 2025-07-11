import numpy as np
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class NoiseInjector:
    def __init__(self, mean=0.0, std_dev=0.3, time_correlation=0.8):
        self.mean = mean
        self.std_dev = std_dev
        self.time_correlation = time_correlation
        self.last_noise = np.zeros(3)

    def generate_noise(self):
        # Generate new random noise
        white_noise = np.random.normal(self.mean, self.std_dev, 3)

        # Apply time correlation using AR(1) process
        self.last_noise = self.time_correlation * self.last_noise + \
                          np.sqrt(1 - self.time_correlation ** 2) * white_noise

        return self.last_noise


class NoisyDronePosition(Node):
    def __init__(self, node_name, noise_params=None):
        super().__init__(node_name)

        if noise_params is None:
            noise_params = {
                'position': {'mean': 0.0, 'std_dev': 0.3, 'time_correlation': 0.8},
                'velocity': {'mean': 0.0, 'std_dev': 0.1, 'time_correlation': 0.6}
            }

        self.position_noise = NoiseInjector(**noise_params['position'])
        self.velocity_noise = NoiseInjector(**noise_params['velocity'])

    def apply_noise_to_position(self, x, y, z):
        position_noise = self.position_noise.generate_noise()
        noisy_position = np.array([x, y, z]) + position_noise

        # Log position deviation
        deviation = np.linalg.norm(position_noise)
        self.get_logger().info(f'Position deviation: {deviation:.2f}m')

        return tuple(noisy_position)

    def apply_noise_to_command(self, cmd_x, cmd_y, cmd_z):
        velocity_noise = self.velocity_noise.generate_noise()
        noisy_command = np.array([cmd_x, cmd_y, cmd_z]) + velocity_noise
        return tuple(noisy_command)

    def log_deviation_stats(self, actual_pos, target_pos):
        deviation = np.linalg.norm(np.array(actual_pos) - np.array(target_pos))
        self.get_logger().info(f'Target deviation: {deviation:.2f}m')
        return deviation