

class World:
    def __init__(self) -> None:
        self.banderole_deployed = False # is the banderole already deplayed ?
        self.MATCH_DURATION = 85        # match duration
        self.enemy_pos = None           # enemy position if known, else None
        