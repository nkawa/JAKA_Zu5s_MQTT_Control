from pyDHgripper import AG95


class ExtendedAG95(AG95):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def read_force(self):
        '''Read the force of the gripper.

        Args:
            None

        Returns:
            rdata: The data read from the serial port.
        '''

        rdata = self.write_uart(modbus_high_addr=0x01, 
                                modbus_low_addr=0x01, 
                                is_set=False)
        return rdata
