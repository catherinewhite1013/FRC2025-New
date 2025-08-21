import math
from commands2 import Subsystem, Command, InstantCommand
from wpilib import SmartDashboard, DigitalInput
from wpimath.filter import Debouncer
from rev import SparkMax, SparkMaxConfig, ClosedLoopConfig
from constants import ShooterConstants

class Shooter(Subsystem):
  def __init__(self):
    self.pitchMotor = SparkMax(
      ShooterConstants.kPitchMotorId, SparkMax.MotorType.kBrushless
    )
    self.rollerMotor = SparkMax(
      ShooterConstants.kRollerMotorId, SparkMax.MotorType.kBrushless
    )

    self.pitchRelEncoder = self.pitchMotor.getEncoder()
    self.pitchAbsEncoder = self.pitchMotor.getAbsoluteEncoder()
    self.pitchClosedLoopController = self.pitchMotor.getClosedLoopController()

    self.rollerEncoder = self.rollerMotor.getEncoder()
    self.rollerClosedLoopController = self.rollerMotor.getClosedLoopController()

    self.configurePitchParam()
    self.configureRollerParam()
    self.resetEncoders()

    self.desiredPitch = float(0)
    self.coralSensor = DigitalInput(11)
    self.coralSensorEnabled = True
    self.debouncer = Debouncer(0.02, Debouncer.DebounceType.kBoth)

  def configurePitchParam(self):
    cfg_pitch = SparkMaxConfig()
    cfg_pitch.setIdleMode(SparkMaxConfig.IdleMode.kBrake)

    cfg_pitch.encoder.positionConversionFactor(ShooterConstants.kPitchEncoderRot2Deg)
    cfg_pitch.encoder.velocityConversionFactor(ShooterConstants.kPitchEncoderRot2Deg / 60)

    cfg_pitch.absoluteEncoder.zeroCentered(True)
    cfg_pitch.absoluteEncoder.zeroOffset(ShooterConstants.kPitchAbsoluteEncoderOffset)
    cfg_pitch.absoluteEncoder.positionConversionFactor(360)
    cfg_pitch.absoluteEncoder.velocityConversionFactor(6)

    cfg_pitch.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
    cfg_pitch.closedLoop.P(ShooterConstants.kPPitchMotor)
    cfg_pitch.closedLoop.D(0)
    cfg_pitch.closedLoop.outputRange(
      -ShooterConstants.kPitchMaxOutput, ShooterConstants.kPitchMaxOutput
    )

    self.pitchMotor.configure(
      cfg_pitch,
      SparkMax.ResetMode.kResetSafeParameters,
      SparkMax.PersistMode.kPersistParameters
    )

  def configureRollerParam(self):
    cfg_roller = SparkMaxConfig()

    self.rollerMotor.configure(
      cfg_roller,
      SparkMax.ResetMode.kResetSafeParameters,
      SparkMax.PersistMode.kPersistParameters
    )

  def resetEncoders(self):
    self.pitchRelEncoder.setPosition(self.pitchAbsEncoder.getPosition())

  def getPitch(self):
    return self.pitchRelEncoder.getPosition()
  
  def setGoalPitch(self, pitch: float):
    self.desiredPitch = pitch
    self.pitchClosedLoopController.setReference(pitch, SparkMax.ControlType.kPosition)

  def atGoalPitch(self):
    return math.isclose(self.getPitch(), self.desiredPitch, abs_tol=10)

  def stopPitch(self):
    self.pitchMotor.stopMotor()

  def intakeCoralCommand(self, reversed: bool) -> Command:
    return InstantCommand(lambda: self.rollerMotor.set(-0.25 if reversed else 0.25))

  def outtakeCoralCommand(self, slowdown: bool) -> Command:
    return InstantCommand(lambda: self.rollerMotor.set(0.57 if slowdown else 1))

  def isCoralFilled(self):
    return (not self.coralSensor.get()) and self.coralSensorEnabled

  def setCoralSensorEnabled(self, enabled: bool):
    self.coralSensorEnabled = enabled

  def stopRoller(self):
    self.rollerMotor.stopMotor()

  def stop(self):
    self.stopPitch()
    self.stopRoller()

  def periodic(self):
    SmartDashboard.putNumber("Shooter Pitch", self.getPitch())
    SmartDashboard.putBoolean("Coral Filled", self.isCoralFilled())
    SmartDashboard.putBoolean("Coral Sensor Enabled", self.coralSensorEnabled)
    if abs(self.getPitch() - self.pitchAbsEncoder.getPosition()) >= 5: self.resetEncoders()
