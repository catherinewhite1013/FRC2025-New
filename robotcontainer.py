from wpilib import SmartDashboard, DriverStation
from wpimath.units import degreesToRadians
from commands import DriveJoystick, GotoPreset, IntakeCoral, ScoreCoral
from commands2 import Command, InstantCommand, ParallelCommandGroup, WaitCommand
from commands2.button import CommandXboxController
from pathplannerlib.auto import AutoBuilder, PathPlannerAuto
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import PIDConstants
from pathplannerlib.path import PathPlannerPath, PathConstraints
from subsystems import Swerve, Elevator, Shooter, Intake
from constants import AutoConstants, OIConstants, MotionPresets

class RobotContainer:
  def __init__(self):
    self.swerve = Swerve()
    self.elevator = Elevator()
    self.shooter = Shooter()
    self.intake = Intake()

    self.driverJoystick = CommandXboxController(OIConstants.kDriverControllerPort)
    self.operatorJoystick = CommandXboxController(OIConstants.kOperatorControllerPort)

    AutoBuilder.configure(
      self.swerve.getPose,
      self.swerve.resetPose,
      self.swerve.getChassisSpeeds,
      lambda speed, _: self.swerve.driveRobotRelative(speed),
      PPHolonomicDriveController(
        PIDConstants(AutoConstants.kPTranslation),
        PIDConstants(AutoConstants.kPRotation)
      ),
      AutoConstants.kRobotConfig,
      self.shouldFlipPath,
      self.swerve
    )

    self.configureButtonBindings()
    SmartDashboard.putString("Starting Point?", "")
    SmartDashboard.putString("Auto Endpoint Selector", "")

    self.canAutoPathfind = False

    self.swerve.setDefaultCommand(
      DriveJoystick(
        self.swerve,
        lambda: -self.driverJoystick.getLeftY(),
        lambda: -self.driverJoystick.getLeftX(),
        lambda: -self.driverJoystick.getRightX()
      )
    )

  def configureButtonBindings(self):
    self.driverJoystick.x().and_(self.driverJoystick.povDown()).and_(
      lambda: self.canAutoPathfind).onTrue(
      self.getPathfindThenFollowPathCommand("Reef A")
    )
    self.driverJoystick.b().and_(self.driverJoystick.povDown()).and_(
      lambda: self.canAutoPathfind).onTrue(
      self.getPathfindThenFollowPathCommand("Reef B")
    )
    self.driverJoystick.x().and_(self.driverJoystick.povDownRight()).and_(
      lambda: self.canAutoPathfind).onTrue(
      self.getPathfindThenFollowPathCommand("Reef C")
    )
    self.driverJoystick.b().and_(self.driverJoystick.povDownRight()).and_(
      lambda: self.canAutoPathfind).onTrue(
      self.getPathfindThenFollowPathCommand("Reef D")
    )
    self.driverJoystick.x().and_(self.driverJoystick.povUpRight()).and_(
      lambda: self.canAutoPathfind).onTrue(
      self.getPathfindThenFollowPathCommand("Reef E")
    )
    self.driverJoystick.b().and_(self.driverJoystick.povUpRight()).and_(
      lambda: self.canAutoPathfind).onTrue(
      self.getPathfindThenFollowPathCommand("Reef F")
    )
    self.driverJoystick.x().and_(self.driverJoystick.povUp()).and_(
      lambda: self.canAutoPathfind).onTrue(
      self.getPathfindThenFollowPathCommand("Reef G")
    )
    self.driverJoystick.b().and_(self.driverJoystick.povUp()).and_(
      lambda: self.canAutoPathfind).onTrue(
      self.getPathfindThenFollowPathCommand("Reef H")
    )
    self.driverJoystick.x().and_(self.driverJoystick.povUpLeft()).and_(
      lambda: self.canAutoPathfind).onTrue(
      self.getPathfindThenFollowPathCommand("Reef I")
    )
    self.driverJoystick.b().and_(self.driverJoystick.povUpLeft()).and_(
      lambda: self.canAutoPathfind).onTrue(
      self.getPathfindThenFollowPathCommand("Reef J")
    )
    self.driverJoystick.x().and_(self.driverJoystick.povDownLeft()).and_(
      lambda: self.canAutoPathfind).onTrue(
      self.getPathfindThenFollowPathCommand("Reef K")
    )
    self.driverJoystick.b().and_(self.driverJoystick.povDownLeft()).and_(
      lambda: self.canAutoPathfind).onTrue(
      self.getPathfindThenFollowPathCommand("Reef L")
    )
    self.driverJoystick.y().and_(self.driverJoystick.leftBumper()).and_(
      lambda: self.canAutoPathfind).onTrue(
      ParallelCommandGroup(
        self.getPathfindThenFollowPathCommand("Left Coral Station"),
        GotoPreset(self.elevator, self.shooter, MotionPresets.CORAL_STATION)
      )
    )
    self.driverJoystick.y().and_(self.driverJoystick.rightBumper()).and_(
      lambda: self.canAutoPathfind).onTrue(
      ParallelCommandGroup(
        self.getPathfindThenFollowPathCommand("Right Coral Station"),
        GotoPreset(self.elevator, self.shooter, MotionPresets.CORAL_STATION)
      )
    )
    self.driverJoystick.a().onTrue(InstantCommand(self.swerve.stop, self.swerve))

    self.operatorJoystick.start().onTrue(
      GotoPreset(self.elevator, self.shooter, MotionPresets.CORAL_STATION)
    )
    self.operatorJoystick.back().onTrue(
      GotoPreset(self.elevator, self.shooter, MotionPresets.HOME)
    )
    self.operatorJoystick.povUp().onTrue(
      GotoPreset(self.elevator, self.shooter, MotionPresets.SCORE_L1)
    )
    self.operatorJoystick.povRight().onTrue(
      GotoPreset(self.elevator, self.shooter, MotionPresets.SCORE_L2)
    )
    self.operatorJoystick.povDown().onTrue(
      GotoPreset(self.elevator, self.shooter, MotionPresets.SCORE_L3)
    )
    self.operatorJoystick.leftTrigger().onTrue(
      InstantCommand(lambda: self.shooter.setCoralSensorEnabled(False))
    )
    self.operatorJoystick.rightTrigger().onTrue(
      InstantCommand(lambda: self.shooter.setCoralSensorEnabled(True))
    )
    self.operatorJoystick.leftStick().onTrue(InstantCommand(self.stopAll))
    self.operatorJoystick.a().onTrue(
      ScoreCoral(self.shooter)
    )
    self.operatorJoystick.b().whileTrue(
      IntakeCoral(self.shooter, self.intake).until(self.shooter.isCoralFilled)
    )
    self.operatorJoystick.x().whileTrue(
      IntakeCoral(self.shooter, self.intake, True)
    )

  def shouldFlipPath(self):
    alliance = DriverStation.getAlliance()
    return False if alliance is None else alliance == DriverStation.Alliance.kRed

  def stopAll(self):
    self.swerve.stop()
    self.elevator.stop()
    self.shooter.stop()
    self.intake.stop()

  def getAutonomousCommand(self) -> Command:
    selected = str(SmartDashboard.getString("Starting Point?", "")).strip().lower()
    if selected not in "lmr" or len(selected) != 1: return None
    autoCommand = PathPlannerAuto({"l": "Left", "m": "Middle", "r": "Right"}[selected] + " Start")
    selected = str(SmartDashboard.getString("Auto Endpoint Selector", "")).strip()
    self.canAutoPathfind = True
    if not selected.isnumeric(): return autoCommand
    keyMapping = {"1": "AB", "2": "CD", "3": "EF", "4": "GH", "5": "IJ", "6": "KL"}
    if keyMapping.get(selected) == None: return autoCommand
    return autoCommand.andThen(
      GotoPreset(self.elevator, self.shooter, MotionPresets.SCORE_L1),
      self.getPathfindThenFollowPathCommand("Reef " + keyMapping.get(selected[0])),
      WaitCommand(0.5),
      ScoreCoral(self.shooter, True)
    )

  def getPathfindThenFollowPathCommand(self, pathName: str):
    path = PathPlannerPath.fromPathFile(pathName)
    constraints = PathConstraints(
      3, 2,
      degreesToRadians(540), degreesToRadians(360)
    )
    return AutoBuilder.pathfindThenFollowPath(path, constraints)
