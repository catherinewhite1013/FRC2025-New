import math
import typing
from wpilib.simulation import DCMotorSim, SimDeviceSim, ElevatorSim, RoboRioSim, BatterySim
from wpilib import DriverStation, SmartDashboard, Mechanism2d
from wpimath.units import radiansToRotations
from wpimath.system.plant import DCMotor, LinearSystemId
from pyfrc.physics.core import PhysicsInterface
from phoenix6 import unmanaged
from rev import SparkMaxSim
from swervemodule import SwerveModule
from subsystems.swerve import Swerve
from subsystems.elevator import Elevator
from constants import DriveConstants, ElevatorConstants, PhysicsConstants

if typing.TYPE_CHECKING:
  from robot import MyRobot

class SwerveModulePhysics:
  def __init__(self, module: SwerveModule):
    self.module = module
    self.driveMotorSim = module.driveMotor.sim_state
    self.steerMotorSim = module.steerMotor.sim_state
    self.shaftEncoderSim = module.shaftEncoder.sim_state

    gearbox = DCMotor.falcon500(1)
    self.driveMotorSysId = DCMotorSim(
      LinearSystemId.DCMotorSystem(gearbox, PhysicsConstants.kDriveMotorMOI, 1), gearbox
    )
    self.steerMotorSysId = DCMotorSim(
      LinearSystemId.DCMotorSystem(gearbox, PhysicsConstants.kSteerMotorMOI, 1), gearbox
    )

  def update(self, tm_diff: float):
    self.driveMotorSim.set_supply_voltage(RoboRioSim.getVInVoltage())
    self.steerMotorSim.set_supply_voltage(RoboRioSim.getVInVoltage())
    self.driveMotorSysId.setInputVoltage(self.driveMotorSim.motor_voltage)
    self.steerMotorSysId.setInputVoltage(self.steerMotorSim.motor_voltage)

    self.driveMotorSysId.update(tm_diff)
    self.steerMotorSysId.update(tm_diff)
    self.updateKinematic()

  def updateKinematic(self):
    self.driveMotorSim.set_raw_rotor_position(radiansToRotations(self.driveMotorSysId.getAngularPosition()))
    self.shaftEncoderSim.set_raw_position(radiansToRotations(self.steerMotorSysId.getAngularPosition()))

    self.driveMotorSim.set_rotor_velocity(radiansToRotations(self.driveMotorSysId.getAngularVelocity()))
    self.shaftEncoderSim.set_velocity(radiansToRotations(self.steerMotorSysId.getAngularVelocity()))

  def getDriveDutyCycle(self):
    return self.module.driveMotor.get_duty_cycle().value_as_double
  
  def getCurrentDraw(self):
    return [self.driveMotorSysId.getCurrentDraw(), self.steerMotorSysId.getCurrentDraw()]

class SwervePhysics:
  def __init__(self, swerve: Swerve):
    self.swerve = swerve
    self.modulesSim = [SwerveModulePhysics(m) for m in self.swerve.modules]
    self.gyroAngleSim = SimDeviceSim(PhysicsConstants.kGyroSimDevice).getDouble("Yaw")
  
  def drive(self, tm_diff: float):
    for m in self.modulesSim:
      m.update(tm_diff)

    chassisSpeeds = self.swerve.getChassisSpeeds()
    if abs(chassisSpeeds.omega) > DriveConstants.kDeadband:
      self.gyroAngleSim.set(
        -(self.swerve.getRawRotation2d().degrees() + chassisSpeeds.omega_dps * tm_diff)
      )

  def getCurrentDraw(self):
    currentDraw = []
    for m in self.modulesSim:
      currentDraw += m.getCurrentDraw()
    return currentDraw

class ElevatorPhysics:
  def __init__(self, elevator: Elevator):
    self.elevator = elevator

    gearbox = DCMotor.NEO(2)
    self.elevatorSim = ElevatorSim(
      gearbox,
      1 / ElevatorConstants.kMotorGearRatio,
      PhysicsConstants.kElevatorMassKilograms,
      ElevatorConstants.kChainPitchMeters * ElevatorConstants.kSprocketTeeth / math.pi,
      ElevatorConstants.kReverseLimitMeters,
      ElevatorConstants.kForwardLimitMeters,
      PhysicsConstants.kElevatorSimGravity,
      ElevatorConstants.kReverseLimitMeters,
      [PhysicsConstants.kLiftMotorStdDevs, 0.0]
    )
    self.liftMotorSim = SparkMaxSim(self.elevator.motor, gearbox)

    self.mechanism = Mechanism2d(20, 50)
    self.elevatorRoot = self.mechanism.getRoot("Elevator Root", 10, 0)
    self.elevatorMechanism = self.elevatorRoot.appendLigament(
      "Elevator", self.elevatorSim.getPositionInches(), 90
    )
    SmartDashboard.putData("Elevator Sim", self.mechanism)

  def update(self, tm_diff: float):
    self.elevatorSim.setInput(
      0, self.liftMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage()
    )
    self.elevatorSim.update(tm_diff)

    self.liftMotorSim.iterate(self.elevatorSim.getVelocity(), RoboRioSim.getVInVoltage(), tm_diff)
    self.elevatorMechanism.setLength(self.elevatorSim.getPositionInches())

  def getCurrentDraw(self):
    return [self.elevatorSim.getCurrentDraw()]

class PhysicsEngine:
  def __init__(
    self, physics_controller: PhysicsInterface, robot: "MyRobot"
  ):
    self.swerveSim = SwervePhysics(robot.swerve)
    self.elevatorSim = ElevatorPhysics(robot.elevator)
  
  def update_sim(self, now: float, tm_diff: float):
    if DriverStation.isEnabled():
      unmanaged.feed_enable(100)

    self.swerveSim.drive(tm_diff)
    self.elevatorSim.update(tm_diff)

    RoboRioSim.setVInVoltage(
      BatterySim.calculate(self.swerveSim.getCurrentDraw() + self.elevatorSim.getCurrentDraw())
    )