import math
from constants import ModuleConstants, DriveConstants
from phoenix6 import BaseStatusSignal
from phoenix6.configs import CANcoderConfiguration, TalonFXConfiguration
from phoenix6.configs.config_groups import SensorDirectionValue
from phoenix6.configs.config_groups import InvertedValue, NeutralModeValue
from phoenix6.controls import StaticBrake, VelocityVoltage, PositionDutyCycle
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.signals import FeedbackSensorSourceValue
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState

class SwerveModule:
  def __init__(
    self,
    driveMotorId: int,
    steerMotorId: int,
    shaftEncoderId: int,
    shaftEncoderOffset: float
  ):
    self.driveMotor = TalonFX(driveMotorId, ModuleConstants.kCANbus)
    self.steerMotor = TalonFX(steerMotorId, ModuleConstants.kCANbus)
    self.shaftEncoder = CANcoder(shaftEncoderId, ModuleConstants.kCANbus)
    self.shaftEncoderOffset = shaftEncoderOffset

    self.configureBaseParam()
    self.resetEncoders()
    
    self.drivePositionSignal = self.driveMotor.get_position()
    self.steerPositionSignal = self.steerMotor.get_position()
    self.driveVelocitySignal = self.driveMotor.get_velocity()
    self.steerVelocitySignal = self.steerMotor.get_velocity()

    BaseStatusSignal.set_update_frequency_for_all(
      250,
      self.drivePositionSignal,
      self.steerPositionSignal,
      self.driveVelocitySignal,
      self.steerVelocitySignal
    )

    self.driveVelocityVoltage = VelocityVoltage(0).with_slot(0)
    self.steerPositionDutyCycle = PositionDutyCycle(0).with_slot(0)
  
  def configureBaseParam(self):
    cfg_drive = TalonFXConfiguration()
    cfg_drive.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
    cfg_drive.feedback.sensor_to_mechanism_ratio = 1 / ModuleConstants.kDriveEncoderRot2Meter
    cfg_drive.slot0.k_p = ModuleConstants.kPDriveMotor
    cfg_drive.slot0.k_s = ModuleConstants.kSDriveMotor
    cfg_drive.slot0.k_v = ModuleConstants.kVDriveMotor
    self.driveMotor.configurator.apply(cfg_drive)

    cfg_steer = TalonFXConfiguration()
    cfg_steer.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
    cfg_steer.motor_output.neutral_mode = NeutralModeValue.BRAKE
    cfg_steer.feedback.feedback_remote_sensor_id = self.shaftEncoder.device_id
    cfg_steer.feedback.feedback_sensor_source = FeedbackSensorSourceValue.REMOTE_CANCODER
    cfg_steer.closed_loop_general.continuous_wrap = True
    cfg_steer.slot0.k_p = ModuleConstants.kPSteerMotor
    self.steerMotor.configurator.apply(cfg_steer)

    cfg_shaft = CANcoderConfiguration()
    cfg_shaft.magnet_sensor.absolute_sensor_discontinuity_point = 1
    cfg_shaft.magnet_sensor.sensor_direction = SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
    cfg_shaft.magnet_sensor.magnet_offset = -self.shaftEncoderOffset
    self.shaftEncoder.configurator.apply(cfg_shaft)
    
  def getDrivePosition(self):
    return self.drivePositionSignal.refresh().value_as_double

  def getSteerPosition(self):
    return self.steerPositionSignal.refresh().value * math.tau
  
  def getDriveVelocity(self):
    return self.driveVelocitySignal.refresh().value_as_double

  def getSteerVelocity(self):
    return self.steerVelocitySignal.refresh().value * math.tau
  
  def resetEncoders(self):
    self.driveMotor.set_position(0)
  
  def getModuleAngle(self):
    return self.getModuleState().angle

  def getModulePosition(self):
    return SwerveModulePosition(self.getDrivePosition(), Rotation2d(self.getSteerPosition()))
  
  def getModuleState(self):
    return SwerveModuleState(self.getDriveVelocity(), Rotation2d(self.getSteerPosition()))

  def setDesiredState(self, state: SwerveModuleState):
    if state.speed <= DriveConstants.kDeadband:
      self.stop()
      return
    state.optimize(self.getModuleAngle())
    self.driveMotor.set_control(self.driveVelocityVoltage.with_velocity(state.speed))
    self.steerMotor.set_control(
      self.steerPositionDutyCycle.with_position(state.angle.radians() / math.tau)
    )

  def stop(self):
    self.driveMotor.set_control(StaticBrake())
    self.steerMotor.set_control(StaticBrake())
