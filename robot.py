from wpilib import Timer, XboxController
from commands2 import TimedCommandRobot, SequentialCommandGroup, InstantCommand, WaitCommand
from commands2.button import Trigger
from robotcontainer import RobotContainer

class MyRobot(TimedCommandRobot):
  def robotInit(self):
    self.container = RobotContainer()
    self.swerve = self.container.swerve
    self.elevator = self.container.elevator
    self.driverJoystick = self.container.driverJoystick

    self.timer = Timer()
    Trigger(lambda: self.timer.hasElapsed(85)).onTrue(
      SequentialCommandGroup(
        InstantCommand(
          lambda: self.driverJoystick.setRumble(XboxController.RumbleType.kBothRumble, 1)
        ),
        WaitCommand(5),
        InstantCommand(
          lambda: self.driverJoystick.setRumble(XboxController.RumbleType.kBothRumble, 0)
        )
      )
    )

  def autonomousInit(self):
    self.autonomousCommand = self.container.getAutonomousCommand()
    if self.autonomousCommand:
      self.autonomousCommand.schedule()

  def teleopInit(self):
    self.timer.restart()

  def disabledInit(self):
    self.container.stopAll()
    self.timer.stop()
