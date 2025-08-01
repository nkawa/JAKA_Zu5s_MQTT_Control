from pyDHgripper import AG95


if __name__ == '__main__':
    """
    # prints are shown in stdout at once at last, why?
    # set_pos seems not blocking
    # set_pos is done correctly

    gripper.read_state()=1
    gripper.read_pos()=1000
    gripper.set_pos(0)=None
    gripper.read_pos()=994
    gripper.set_pos(1000)=None
    gripper.read_pos()=977
    """
    check_move = True
    gripper = AG95(port='/dev/ttyUSB0')
    print(f"{gripper.read_state()=}")
    print(f"{gripper.read_pos()=}")
    # Grip
    if check_move:
        print(f"{gripper.set_pos(0)=}")
    print(f"{gripper.read_pos()=}")
    # Release
    if check_move:
        print(f"{gripper.set_pos(1000)=}")
    print(f"{gripper.read_pos()=}")
