from pyDHgripper import AG95


if __name__ == '__main__':
    gripper = AG95(port='/dev/ttyUSB0')
    print(gripper.read_state())
    print(gripper.read_pos())
    # Grip
    print(gripper.set_pos(0))
    print(gripper.read_pos())
    # Release
    print(gripper.set_pos(1000))
    print(gripper.read_pos())
    