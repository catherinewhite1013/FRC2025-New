from commands2 import SequentialCommandGroup
from subsystems import Elevator, Shooter
from commands.setelevatorheight import SetElevatorHeight
from commands.setshooterpitch import SetShooterPitch
from constants import MotionPresets

class GotoPreset(SequentialCommandGroup):
  def __init__(self, elevator: Elevator, shooter: Shooter, preset: MotionPresets):
    super().__init__()

    self.addCommands(
      SetShooterPitch(shooter, preset.value[1]).withTimeout(1),
      SetElevatorHeight(elevator, preset.value[0]).withTimeout(2)
    )
