from commands2 import SequentialCommandGroup, WaitCommand
from subsystems import Shooter, Intake

class IntakeCoral(SequentialCommandGroup):
  def __init__(self, shooter: Shooter, intake: Intake, reversed = False):
    super().__init__()

    self.shooter = shooter
    self.intake = intake
    self.addCommands(
      shooter.intakeCoralCommand(reversed),
      intake.intakeCoralCommand(),
      WaitCommand(5)
    )

    self.addRequirements(shooter, intake)

  def end(self, interrupted):
    self.shooter.stopRoller()
    self.intake.stop()
