from jaka_control.dummy_hand_control import DummyHandControl


tool_infos = [
    {
        "id": 1,
        "name": "dhrobotics_ag95",
    }
]
tool_classes = {
    # NOTE: 本来はAG95Controlにしてもよいが、
    # VRからどのような制御値が来るかわからないので現在は使用していない
    "dhrobotics_ag95": DummyHandControl
}
tool_base = []
