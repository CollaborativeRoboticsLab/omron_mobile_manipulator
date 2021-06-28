import rclpy
import time
import sys
import json
from ament_index_python.packages import get_package_share_directory
pp_share = get_package_share_directory('pickplace')
pp_library =  pp_share + '/pickplace/pp_library'

from pp_library import Modbus, Transform, Script, Move



def main():
    rclpy.init()
    modbus = Modbus.ModbusClass()
    modbus.open_io()



if __name__ == '__main__':
    main()

