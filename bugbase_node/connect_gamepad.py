import rclpy
from rclpy.node import Node

from .bluetoothctl_wrapper import Bluetoothctl, BluetoothctlError


class BluetoothctlNode(Node):

    def __init__(self):
        super().__init__('autoconnect_gamepad')
        self.bl = Bluetoothctl()
        self.connect_gamepad_timer = self.create_timer(1.0 , self.connect_gamepad)

    def connect_gamepad(self) -> None:
        paired_devices = self.bl.get_paired_devices()
        
        if not paired_devices:
            return
        for device in paired_devices:
            if device['mac_address'] == b"83:33:A0:50:31:46":
                return
        
        self.bl.connect("83:33:A0:50:31:46")

def main(args=None):
    rclpy.init(args=args)

    bt = BluetoothctlNode()

    rclpy.spin(bt)
    bt.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

