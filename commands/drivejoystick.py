import commands2
from typing import Callable
from constants import DriveConstants, OIConstants
from wpimath import applyDeadband
from wpimath.filter import SlewRateLimiter
from wpimath.kinematics import ChassisSpeeds
from subsystems import Swerve

class DriveJoystick(commands2.Command):
  def __init__(
      self,
      swerve: Swerve,
      xSpeed: Callable[[], float],
      ySpeed: Callable[[], float],
      oSpeed: Callable[[], float],
  ):
    self.swerve = swerve
    self.xSpeed = xSpeed
    self.ySpeed = ySpeed
    self.oSpeed = oSpeed

    self.xLimiter = SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond)
    self.yLimiter = SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond)
    self.oLimiter = SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond)

    self.addRequirements(self.swerve)

  def execute(self):
    rawValues = [self.xSpeed(), self.ySpeed(), self.oSpeed()]
    xSpeed, ySpeed, oSpeed = map(lambda v: applyDeadband(v, OIConstants.kDeadband), rawValues)

    xSpeed = self.xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond
    ySpeed = self.yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond
    oSpeed = self.oLimiter.calculate(oSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond

    #chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
    #  xSpeed, ySpeed, oSpeed, self.swerve.getRotation2d()
    #)
    chassisSpeeds = ChassisSpeeds(xSpeed, ySpeed, oSpeed)
    
    self.swerve.driveRobotRelative(chassisSpeeds)

  def end(self, interrupted):
    self.swerve.stop()

  def isFinished(self):
    return False
