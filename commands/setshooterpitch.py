from commands2 import Command
from subsystems import Shooter

class SetShooterPitch(Command):
  def __init__(self, shooter: Shooter, pitch: float):
    self.shooter = shooter
    self.pitch = pitch

    self.addRequirements(self.shooter)
    
  def initialize(self):
    self.shooter.setGoalPitch(self.pitch)

  def isFinished(self):
    return self.shooter.atGoalPitch()
