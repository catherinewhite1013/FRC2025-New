from commands2 import SequentialCommandGroup, WaitCommand
from subsystems import Shooter

class ScoreCoral(SequentialCommandGroup):
  def __init__(self, shooter: Shooter, slowdown = False):
    super().__init__()

    self.shooter = shooter
    self.addCommands(
      shooter.outtakeCoralCommand(slowdown),
      WaitCommand(1)
    )

    self.addRequirements(shooter)

  def end(self, interrupted):
    self.shooter.stopRoller()