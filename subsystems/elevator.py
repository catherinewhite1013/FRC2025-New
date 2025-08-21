import math
import commands2
from wpilib import SmartDashboard
from rev import SparkMax, SparkMaxConfig, LimitSwitchConfig, ClosedLoopConfig
from constants import ElevatorConstants

class Elevator(commands2.Subsystem):
  def __init__(self):
    self.motor = SparkMax(
      ElevatorConstants.kMotorId, SparkMax.MotorType.kBrushless
    )
    self.motorSub = SparkMax(
      ElevatorConstants.kMotorSubId, SparkMax.MotorType.kBrushless
    )

    self.encoder = self.motor.getEncoder()
    self.closedLoopController = self.motor.getClosedLoopController()

    cfg = SparkMaxConfig()
    cfg.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
    cfg.smartCurrentLimit(ElevatorConstants.kSmartCurrentLimit)

    cfg.encoder.positionConversionFactor(ElevatorConstants.kEncoderRot2Meter)
    cfg.encoder.velocityConversionFactor(ElevatorConstants.kEncoderRot2Meter / 60)

    cfg.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
    cfg.limitSwitch.forwardLimitSwitchEnabled(True)
    cfg.limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
    cfg.limitSwitch.reverseLimitSwitchEnabled(True)
    cfg.softLimit.forwardSoftLimit(ElevatorConstants.kForwardLimitMeters)
    cfg.softLimit.forwardSoftLimitEnabled(True)
    cfg.softLimit.reverseSoftLimit(ElevatorConstants.kReverseLimitMeters)
    cfg.softLimit.reverseSoftLimitEnabled(True)

    cfg.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
    cfg.closedLoop.P(ElevatorConstants.kPMotorPosition)
    cfg.closedLoop.D(0.01)
    cfg.closedLoop.outputRange(
      -ElevatorConstants.kMaxOutput, ElevatorConstants.kMaxOutput
    )

    self.motor.configure(
      cfg,
      SparkMax.ResetMode.kResetSafeParameters,
      SparkMax.PersistMode.kPersistParameters
    )

    self.motorSub.configure(
      SparkMaxConfig().apply(cfg).follow(self.motor, True),
      SparkMax.ResetMode.kResetSafeParameters,
      SparkMax.PersistMode.kPersistParameters
    )

    self.resetEncoders()
    self.desiredHeight = float(0)

  def resetEncoders(self):
    self.encoder.setPosition(0)

  def getHeight(self):
    return self.encoder.getPosition()

  def setGoalHeight(self, height: float):
    self.desiredHeight = height
    self.closedLoopController.setReference(height, SparkMax.ControlType.kPosition)

  def atGoalHeight(self):
    return math.isclose(self.getHeight(), self.desiredHeight, abs_tol=0.05)

  def stop(self):
    self.motor.stopMotor()

  def periodic(self):
    SmartDashboard.putNumber("Elevator Height", self.getHeight())