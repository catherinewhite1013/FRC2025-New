import math
from enum import Enum
from wpimath.units import inchesToMeters, degreesToRadians
from wpimath.geometry import Translation2d, Transform3d, Rotation3d
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians
from wpimath.kinematics import SwerveDrive4Kinematics
from pathplannerlib.config import RobotConfig
from phoenix6 import CANBus

class ModuleConstants:
  kCANbus = CANBus("canivore")
  kWheelDiameterMeters = 0.097
  kDriveMotorGearRatio = 1 / 6.75
  kDriveEncoderRotToMeter = kDriveMotorGearRatio * math.pi * kWheelDiameterMeters   #talon Fx build in encoder(drive)
  kDriveMotorMaxRPM = 6380
  kDriveMotorNominalVoltage = 12
  kPDriveMotor = 0.95
  kSDriveMotor = 0.06
  kVDriveMotor = kDriveMotorNominalVoltage * 60 / kDriveEncoderRotToMeter / kDriveMotorMaxRPM
  kPSteerMotor = 0.7

class DriveConstants:
  kTrackWidthMeters = 0.7
  kWheelBaseMeters  = 0.53507
  kDriveKinematics = SwerveDrive4Kinematics(
    Translation2d( kWheelBaseMeters / 2,  kWheelBaseMeters / 2),
    Translation2d( kWheelBaseMeters / 2, -kWheelBaseMeters / 2),
    Translation2d(-kWheelBaseMeters / 2,  kWheelBaseMeters / 2),
    Translation2d(-kWheelBaseMeters / 2, -kWheelBaseMeters / 2)
  )

  kDeadband = 0.001
  kPHeading = 1.4

  kPhysicalMaxSpeedMetersPerSecond = 5
  kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * math.tau
  kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 2
  kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4
  kTeleDriveMaxAccelerationUnitsPerSecond = 3.0
  kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3.0

  # whoami
  kFrontLeftDriveMotorId = 0
  kFrontLeftSteerMotorId = 1
  kFrontLeftShaftEncoderId = 10
  kFrontLeftShaftEncoderOffset = 0.903

  kFrontRightDriveMotorId = 2
  kFrontRightSteerMotorId = 3
  kFrontRightShaftEncoderId = 11
  kFrontRightShaftEncoderOffset = 0.9929

  kBackLeftDriveMotorId = 6
  kBackLeftSteerMotorId = 7
  kBackLeftShaftEncoderId = 13
  kBackLeftShaftEncoderOffset = 0.488

  kBackRightDriveMotorId = 4
  kBackRightSteerMotorId = 5
  kBackRightShaftEncoderId = 12
  kBackRightShaftEncoderOffset = 0.1413

class ElevatorConstants:
  kMotorId = 14
  kMotorSubId = 15

  kChainPitchMeters = inchesToMeters(0.25)
  kSprocketTeeth = 26
  kMotorGearRatio = 1 / 20
  kEncoderRot2Meter = kMotorGearRatio * kSprocketTeeth * kChainPitchMeters * 2

  kSmartCurrentLimit = 80
  kForwardLimitMeters = 1.3
  kReverseLimitMeters = 0
  kMaxOutput = 0.5
  kPMotorPosition = 20

class IntakeConstants:
  kMotorId = 21

class ShooterConstants:
  kPitchMotorId = 16
  kRollerMotorId = 17

  kPitchMotorGearRatio = 1 / 4
  kPitchSprocketRatio = 22 / 50
  kPitchEncoderRot2Deg = kPitchMotorGearRatio * kPitchSprocketRatio * 360
  kPitchAbsoluteEncoderOffset = 0.36

  kPitchMaxOutput = 0.7
  kPPitchMotor = 0.002

class MotionPresets(Enum):
  CORAL_STATION = (0.05, -94)
  SCORE_L1 = (0.25, -66)
  SCORE_L2 = (0.35, -75)
  SCORE_L3 = (0.7, -75)
  REEF_L2 = (0.55, -15)
  REEF_L3 = (0.90, -15)
  HOME = (0, -15)

class AutoConstants:
  kRobotConfig = RobotConfig.fromGUISettings()
  kPTranslation = 1.7
  kPRotation = 3.5

class AutoAlignConstants:
  kMaxSpeedMetersPerSecond = 3
  kMaxAngularSpeedRadiansPerSecond = degreesToRadians(540)
  kMaxAccelerationMetersPerSecondSquared = 2
  kMaxAngularAccelerationRadiansPerSecondSquared = math.tau
  kPXController = 1.5
  kPYController = 1.5
  kPOController = 1.5

  kXControllerConstraints = TrapezoidProfile.Constraints(
    kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared
  )
  kYControllerConstraints = TrapezoidProfile.Constraints(
    kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared
  )
  kOControllerConstraints = TrapezoidProfileRadians.Constraints(
    kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared
  )

  kXToleranceMeters = 0.02
  kYToleranceMeters = 0.015
  kOToleranceRadians = degreesToRadians(2)

  kLeftReefTransform = Transform3d(0.35, 0.10, 0, Rotation3d(0, 0, math.pi))
  kRightReefTransform = Transform3d(0.35, -0.10, 0, Rotation3d(0, 0, math.pi))

class OIConstants:
  kDriverControllerPort = 0
  kOperatorControllerPort = 1
  kDeadband = 0.06

class PhysicsConstants:
  kDriveMotorMOI = 0.0007
  kSteerMotorMOI = 0.0001
  kGyroSimDevice = "navX-Sensor[4]"
  kElevatorMassKilograms = 4
  kElevatorSimGravity = False
  kLiftMotorStdDevs = 0.01
