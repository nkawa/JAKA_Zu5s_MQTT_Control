class MockAG95:
    def __init__(self):
        self.pos = 1000
        self.force = 20

    def set_pos(self, pos: int) -> None:
        self.pos = pos

    def set_force(self, val: int) -> None:
        self.force = val

    def read_pos(self) -> int:
        return self.pos
    
    def read_force(self) -> int:
        return self.force
    
    def read_state(self) -> int:
        return 2
