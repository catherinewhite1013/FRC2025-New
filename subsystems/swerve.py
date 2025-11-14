import math
import commands2
from typing import Iterable
from wpilib import SmartDashboard
from wpimath.controller import PIDController
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Rotation2d, Pose2d, Transform2d
from wpimath.kinematics import SwerveModuleState, SwerveDrive4Kinematics
from wpimath.kinematics import ChassisSpeeds
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpimath.units import inchesToMeters, degreesToRadians
from ntcore import NetworkTableInstance
from navx import AHRS
# from swervemodule import SwerveModule
from constants import DriveConstants
from questnav.questnav2 import QuestNav
from constants import ModuleConstants, DriveConstants
from phoenix6 import BaseStatusSignal
from phoenix6.configs import CANcoderConfiguration, TalonFXConfiguration
from phoenix6.configs.config_groups import SensorDirectionValue
from phoenix6.configs.config_groups import InvertedValue, NeutralModeValue
from phoenix6.controls import StaticBrake, VelocityVoltage, PositionDutyCycle
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.signals import FeedbackSensorSourceValue


class Swerve(commands2.Subsystem):
  def __init__(self):
    #   --- whoami ---
    self.frontLeft = SwerveModule(
      DriveConstants.kFrontLeftDriveMotorId,
      DriveConstants.kFrontLeftSteerMotorId,
      DriveConstants.kFrontLeftShaftEncoderId,
      DriveConstants.kFrontLeftShaftEncoderOffset
    )

    self.frontRight = SwerveModule(
      DriveConstants.kFrontRightDriveMotorId,
      DriveConstants.kFrontRightSteerMotorId,
      DriveConstants.kFrontRightShaftEncoderId,
      DriveConstants.kFrontRightShaftEncoderOffset
    )

    self.backLeft = SwerveModule(
      DriveConstants.kBackLeftDriveMotorId,
      DriveConstants.kBackLeftSteerMotorId,
      DriveConstants.kBackLeftShaftEncoderId,
      DriveConstants.kBackLeftShaftEncoderOffset
    )

    self.backRight = SwerveModule(
      DriveConstants.kBackRightDriveMotorId,
      DriveConstants.kBackRightSteerMotorId,
      DriveConstants.kBackRightShaftEncoderId,
      DriveConstants.kBackRightShaftEncoderOffset
    )
    # --- gyro, angle, communication ---    self.modules = [self.frontLeft, self.frontRight, self.backLeft, self.backRight]
    self.gyro = AHRS(AHRS.NavXComType.kMXP_SPI)
    self.gyro.enableBoardlevelYawReset(True)

    self.desiredHeading = float(0)
    self.headingPIDController = PIDController(DriveConstants.kPHeading, 0, 0)
    self.headingPIDController.enableContinuousInput(-math.pi, math.pi)   #讓角度連續（-179，179差2度，不是358度）

    self.poseEstimator = SwerveDrive4PoseEstimator(
      DriveConstants.kDriveKinematics,
      self.getRawRotation2d(),   #gyro read
      self.getModulePositions(),
      Pose2d(),
      (0.05, 0.05, degreesToRadians(5)),  #IMU 誤差
      (0.02, 0.02, degreesToRadians(2))   #Vision 誤差
    )
    #--- 不同板子溝通 ---
    nt = NetworkTableInstance.getDefault()
    self.statePublisher = nt.getStructArrayTopic("/SwerveStates", SwerveModuleState).publish()
    self.posePublisher  = nt.getStructTopic("/SwervePose", Pose2d).publish()

    self.questnav = QuestNav()
    self.quest_to_robot = Transform2d(-0.22, 0, Rotation2d().fromDegrees(180))
    self.questnav.set_pose(Pose2d().transformBy(self.quest_to_robot.inverse()))

  def zeroHeading(self, fieldRotation: Rotation2d):
    self.gyro.setAngleAdjustment(-fieldRotation.degrees() - self.gyro.getYaw())
    self.desiredHeading = self.getRotation2d().radians()
  
  def getRawRotation2d(self):
    return Rotation2d.fromDegrees(-self.gyro.getYaw())

  def getRotation2d(self):
    return self.gyro.getRotation2d()
  
  def resetPose(self, pose: Pose2d):
    self.poseEstimator.resetPose(pose)
    self.questnav.set_pose(pose.transformBy(self.quest_to_robot.inverse()))

  def getPose(self):
    return self.poseEstimator.getEstimatedPosition()

  def getModulePositions(self):
    return tuple(m.getModulePosition() for m in self.modules)
  
  def getModuleStates(self):
    return tuple(m.getModuleState() for m in self.modules)
  
  def setModuleStates(self, desiredState: Iterable[SwerveModuleState]):
    desiredState = SwerveDrive4Kinematics.desaturateWheelSpeeds(
      desiredState, DriveConstants.kPhysicalMaxSpeedMetersPerSecond
    )
    for i in range(4):
      self.modules[i].setDesiredState(desiredState[i])

  def getChassisSpeeds(self):
    return DriveConstants.kDriveKinematics.toChassisSpeeds(self.getModuleStates())

  def driveRobotRelative(self, chassisSpeeds: ChassisSpeeds):
    differ = lambda a, b: abs((a - b + math.pi) % math.tau - math.pi)
    currentHeading = self.getRotation2d().radians()
    if (abs(chassisSpeeds.omega) > DriveConstants.kDeadband
        or differ(self.desiredHeading, currentHeading) > math.pi / 4
    ):
      self.desiredHeading = currentHeading
    elif (chassisSpeeds.vx**2 + chassisSpeeds.vy**2)**0.5 > DriveConstants.kDeadband:
      chassisSpeeds.omega = self.headingPIDController.calculate(currentHeading, self.desiredHeading)

    moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds)
    self.setModuleStates(moduleStates)

  def stop(self):
    for m in self.modules:
      m.stop()

  def periodic(self):
    self.statePublisher.set(list(self.getModuleStates()))
    self.posePublisher.set(self.getPose())
    self.poseEstimator.update(self.getRawRotation2d(), self.getModulePositions())
    SmartDashboard.putNumber("Heading", self.getRotation2d().degrees())
    SmartDashboard.putBoolean("QUEST_CONNECTED", self.questnav.is_connected())
    SmartDashboard.putBoolean("QUEST_TRACKING", self.questnav.is_tracking())

    self.questnav.command_periodic()
    if(self.questnav.is_tracking()):
      quest_pose = self.questnav.get_pose().transformBy(self.quest_to_robot)
      self.poseEstimator.addVisionMeasurement(quest_pose, self.questnav.get_data_timestamp())


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
