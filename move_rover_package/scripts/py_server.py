#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from move_rover_package.module_to_import import SimpleRoverActionServer

def main(args=None):
    rclpy.init(args=args)
    server = SimpleRoverActionServer()
    rclpy.spin(server)


if __name__ == '__main__':
    main()
