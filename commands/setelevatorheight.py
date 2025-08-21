from commands2 import Command
from subsystems import Elevator

class SetElevatorHeight(Command):
  def __init__(self, elevator: Elevator, height: float):
    self.elevator = elevator
    self.height = height

    self.addRequirements(self.elevator)
    
  def initialize(self):
    self.elevator.setGoalHeight(self.height)

  def isFinished(self):
    return self.elevator.atGoalHeight()

  