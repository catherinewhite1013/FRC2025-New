import math
import commands2
from typing import Iterable
from wpilib import SmartDashboard
from wpimath.controller import PIDController
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Rotation2d, Pose2d, Transform2d
from wpimath.kinematics import SwerveModuleState, SwerveDrive4Kinematics
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import inchesToMeters, degreesToRadians
from ntcore import NetworkTableInstance
from navx import AHRS
from swervemodule import SwerveModule
from constants import DriveConstants
from questnav.questnav2 import QuestNav

class Swerve(commands2.Subsystem):
  def __init__(self):
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

    self.modules = [self.frontLeft, self.frontRight, self.backLeft, self.backRight]
    self.gyro = AHRS(AHRS.NavXComType.kMXP_SPI)
    self.gyro.enableBoardlevelYawReset(True)

    self.desiredHeading = float(0)
    self.headingPIDController = PIDController(DriveConstants.kPHeading, 0, 0)
    self.headingPIDController.enableContinuousInput(-math.pi, math.pi)

    self.poseEstimator = SwerveDrive4PoseEstimator(
      DriveConstants.kDriveKinematics,
      self.getRawRotation2d(),
      self.getModulePositions(),
      Pose2d(),
      (0.05, 0.05, degreesToRadians(5)),
      (0.02, 0.02, degreesToRadians(2))
    )

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