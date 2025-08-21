from commands2 import Subsystem, Command, InstantCommand
from phoenix6.hardware import TalonFX
from phoenix6.controls import CoastOut
from constants import IntakeConstants

class Intake(Subsystem):
  def __init__(self):
    self.motor = TalonFX(IntakeConstants.kMotorId)

  # ===== 基本操作 =====
  def intakeCoralCommand(self) -> Command:
    """吃 Coral"""
    return InstantCommand(lambda: self.motor.set(0.25), self)

  def outtakeCoralCommand(self) -> Command:
    """吐 Coral"""
    return InstantCommand(lambda: self.motor.set(-0.5), self)

  def stopCommand(self) -> Command:
    """停止馬達"""
    return InstantCommand(self.stop, self)

  def stop(self):
    """直接呼叫停止"""
    self.motor.set_control(CoastOut())
