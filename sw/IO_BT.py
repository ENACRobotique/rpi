import py_trees
from actionneurs import*

##### Behavior tree pour l'automatisation #####

class Leaf_LiftUpPlanche(py_trees.behaviour.Behaviour):
    def __init__(self, manager:IO_Manager):
        super().__init__(name=f"Lift Up planche")
        self.manager = manager

    def update(self):
        self.manager.liftUpPlanches(True)
        if self.manager.isLiftUp():
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

class Leaf_LiftDownPlanche(py_trees.behaviour.Behaviour):
    def __init__(self, manager:IO_Manager):
        super().__init__(name=f"Lift Down planche")
        self.manager = manager

    def update(self):
        self.manager.liftUpPlanches(False)
        if self.manager.isLiftDown():
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
    
class Leaf_LiftUpConserve(py_trees.behaviour.Behaviour):
    def __init__(self, manager:IO_Manager):
        super().__init__(name=f"Lift Up Conserve")
        self.manager = manager

    def update(self):
        self.manager.liftConserve(True)
        if self.manager.isConserveUp():
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

class Leaf_LiftDownConserve(py_trees.behaviour.Behaviour):
    def __init__(self, manager:IO_Manager):
        super().__init__(name=f"Lift Down Conserve")
        self.manager = manager

    def update(self):
        self.manager.liftConserve(False)
        if self.manager.isConserveDown():
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
    
class Leaf_RentreurIN(py_trees.behaviour.Behaviour):
    def __init__(self, manager:IO_Manager):
        super().__init__(name=f"Rentreur going INSIDE")
        self.manager = manager

    def update(self):
        self.manager.moveRentreur(INSIDE)
        if self.manager.isRentreurIN():
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
    
class Leaf_RentreurOUT(py_trees.behaviour.Behaviour):
    def __init__(self, manager:IO_Manager):
        super().__init__(name=f"Rentreur going OUTSIDE")
        self.manager = manager

    def update(self):
        self.manager.moveRentreur(OUTSIDE)
        if self.manager.isRentreurOUT():
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

class Leaf_GrabHighConserve(py_trees.behaviour.Behaviour):
    def __init__(self, manager:IO_Manager, grab : bool):
        status = "Dropping"
        if grab:
            status = "Grabbing"
        super().__init__(name=f"{status} Rentreur Conserve")
        self.manager = manager
        self.grab = grab

    def update(self):
        self.manager.grabHighConserve(self.grab)
        return py_trees.common.Status.SUCCESS

class Leaf_GrabLowConserve(py_trees.behaviour.Behaviour):
    def __init__(self, manager:IO_Manager, grab : bool):
        status = "Dropping"
        if grab:
            status = "Grabbing"
        super().__init__(name=f"{status} Rentreur Conserve")
        self.manager = manager
        self.grab = grab

    def update(self):
        self.manager.grabLowConserve(self.grab)
        return py_trees.common.Status.SUCCESS

class Leaf_Sleep(py_trees.behaviour.Behaviour):
    def __init__(self, delay):
        super().__init__(name=f"Waiting {delay} second")
        self.delay = delay
        self.startingTime = time.time()

    def update(self):
        if abs(time.time()-self.startingTime)>= self.delay : 
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
    
class Leaf_LockPlanche(py_trees.behaviour.Behaviour):
    def __init__(self, manager:IO_Manager, lock : bool):
        status = "Unlocking"
        if lock:
            status = "Locking"
        super().__init__(name=f"{status} Upper Planche")
        self.manager = manager
        self.lock = lock

    def update(self):
        self.manager.lockPlanche(self.lock)
        return py_trees.common.Status.SUCCESS
    
class Leaf_DeployPince(py_trees.behaviour.Behaviour):
    def __init__(self, manager:IO_Manager):
        super().__init__(name=f"Deploying Pince")
        self.manager = manager

    def update(self):
        self.manager.deployPince(True)
        return py_trees.common.Status.SUCCESS
    
class Leaf_LiftPlancheDelay(py_trees.behaviour.Behaviour):
    def __init__(self, manager:IO_Manager, direction, delay):
        super().__init__(name=f"Deploying Pince")
        self.manager = manager
        self.delay = delay
        self.direction = direction
        self.startingTime = time.time()

    def update(self):
        self.manager.liftPlancheContinu(DOWN)
        if abs(time.time()-self.startingTime)>= self.delay :
            self.manager.liftPlancheContinu(STOP)
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
    
class Leaf_LiftPlancheCalibrate(py_trees.behaviour.Behaviour):
    def __init__(self, manager:IO_Manager):
        super().__init__(name=f"Deploying Pince")
        self.manager = manager

    def update(self):
        self.manager.liftPlancheContinu(UP)
        if  self.manager.liftDCalibrated and self.manager.liftGCalibrated :    
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
    


jerome = IO_Manager() # Temporaire

Node_deployMacon = py_trees.composites.Sequence("Deploiement Macon", True)
Node_deployMacon.add_children([
        Leaf_DeployPince(jerome),
        Leaf_LockPlanche(jerome, False),
        Leaf_GrabHighConserve(jerome,False),
        Leaf_GrabLowConserve(jerome, False),
        Leaf_LiftPlancheDelay(jerome, DOWN, 0.5),
        Leaf_LiftPlancheCalibrate(jerome) # When the button (fdc) will be pressed this will trigger calibration ,
                                          # The lift will then go down and stop moving
])

Node_ramasseGradin = py_trees.composites.Sequence("Rammasser un Gradin", True)
Node_ramasseGradin.add_children([
    Leaf_LiftUpPlanche(jerome),
    Leaf_Sleep(0.2),
    Leaf_GrabHighConserve(jerome, False),
    Leaf_GrabLowConserve(jerome, True),
    Leaf_Sleep(0.3),
    Leaf_LiftUpConserve(jerome),
    Leaf_RentreurOUT(jerome),
    Leaf_GrabHighConserve(jerome, True),
    Leaf_Sleep(1.5),
    Leaf_GrabLowConserve(jerome, False),
    Leaf_Sleep(0.25),
    Leaf_LockPlanche(jerome, True),
    Leaf_LiftDownConserve(jerome),

    Leaf_GrabLowConserve(jerome, True),
    Leaf_Sleep(0.7),

    Leaf_RentreurIN(jerome),
    Leaf_GrabLowConserve(jerome, False)
])

Node_construitGradin = py_trees.composites.Sequence("Construire un Gradin", True)
Node_construitGradin.add_children([
        Leaf_LiftUpPlanche(jerome),
        Leaf_RentreurOUT(jerome),
        Leaf_GrabHighConserve(jerome,False),
        Leaf_Sleep(0.3),
        Leaf_RentreurIN(jerome) ,
        Leaf_LockPlanche(jerome,False),
        Leaf_Sleep(0.5),
        Leaf_GrabLowConserve(jerome, False) 
])        

