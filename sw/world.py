import time

class World:
    def __init__(self) -> None:
        self.banderole_deployed = False # is the banderole already deplayed ?
        self.MATCH_DURATION = 100        # match duration
        self.enemy_pos = None           # enemy position if known, else None
        self.matchStartTime: float = -1      # Match start time. negative if match not started
        self.backInZone = False
        self.Gradin = {"G3":False,"G4":False}
        self.gradin_pousse_pousse = False
    
    def time_left(self) -> float:
        if self.matchStartTime < 0:
            return self.MATCH_DURATION
        else:
            return max(0, self.MATCH_DURATION - (time.time() - self.matchStartTime))
    
    def match_started(self) -> bool:
        return self.matchStartTime >= 0
        