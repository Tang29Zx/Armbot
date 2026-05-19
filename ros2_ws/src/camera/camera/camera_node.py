import rclpy
from rclpy import Node

class Camera(Node):
    def __init__(self):
        pass



def main():
    rclpy.super.__init__('camera')
    node = Camera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown

if __name__ == "__main__":
    main()