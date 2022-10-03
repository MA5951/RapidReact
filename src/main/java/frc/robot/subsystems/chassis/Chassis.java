package frc.robot.subsystems.chassis;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

import com.ma5951.utils.RobotConstants;
import frc.robot.Limelight;
import com.ma5951.utils.autonomous.OdometryHandler;
import com.ma5951.utils.Calculations;
import com.ma5951.utils.Shuffleboard;
import com.ma5951.utils.controllers.PIDController;
import com.revrobotics.ColorSensorV3;

public class Chassis extends SubsystemBase {
  private static Chassis chassis;

  private TalonFX leftFrontMotor;
  private TalonFX leftRearMotor;
  private TalonFX rightFrontMotor;
  private TalonFX rightRearMotor;

  private AHRS navx;

  private PIDController rightVelocityPID;
  private PIDController leftVelocityPID;

  private PIDController anglePIDVision;

  public Shuffleboard chassisShuffleboard;

  private OdometryHandler odometryHandler;

  private double reverse = 1;

  private ColorSensorV3 colorSensorLeft;
  private ColorSensorV3 colorSensorRight;

  public double invert;

  public static Chassis getinstance() {
    if (chassis == null) {
      chassis = new Chassis();
    }
    return chassis;
  }

  private Chassis() {
    chassisShuffleboard = new Shuffleboard(ChassisConstants.KSUBSYSTEM_NAME);
    leftFrontMotor = new TalonFX(PortMap.chassisLeftFrontMotor);
    leftRearMotor = new TalonFX(PortMap.chassisLeftRearMotor);
    rightFrontMotor = new TalonFX(PortMap.chassisRightFrontMotor);
    rightRearMotor = new TalonFX(PortMap.chassisRightRearMotor);

    leftRearMotor.follow(leftFrontMotor);
    rightRearMotor.follow(rightFrontMotor);

    navx = new AHRS(Port.kMXP);
    odometryHandler = new OdometryHandler(this::getLeftDistance, this::getRightDistance, this::getAngle);

    rightVelocityPID = new PIDController(ChassisConstants.KP_MAPATH_RIGHT_VELOCITY,
        ChassisConstants.KI_MAPATH_RIGHT_VELOCITY, ChassisConstants.KD_MAPATH_RIGHT_VELOCITY, 0, 0, -1, 1);
    leftVelocityPID = new PIDController(ChassisConstants.KP_MAPATH_LEFT_VELOCITY,
        ChassisConstants.KI_MAPATH_LEFT_VELOCITY, ChassisConstants.KD_MAPATH_LEFT_VELOCITY, 0, 0, -1, 1);

    anglePIDVision = new PIDController(ChassisConstants.KP_VISION_ANGLE, ChassisConstants.KI_VISION_ANGLE,
        ChassisConstants.KD_VISION_ANGLE, 0, 2, -12, 12);

    anglePIDVision.enableContinuousInput(-ChassisConstants.KANGLE_PID_VISION_SET_INPUTRANGE,
        ChassisConstants.KANGLE_PID_VISION_SET_INPUTRANGE);

    colorSensorLeft = new ColorSensorV3(I2C.Port.kMXP);
    colorSensorRight = new ColorSensorV3(I2C.Port.kOnboard);

    resetSensors();
    rightFrontMotor.setInverted(TalonFXInvertType.Clockwise);
    rightRearMotor.setInverted(TalonFXInvertType.FollowMaster);

    invert = 1;

  }

  /**
   * Invert motors and navX
   * 
   * @param reverse true to invert
   */
  public void setInverted(boolean reverse) {
    if (reverse) {
      this.reverse = 1.0;
      rightFrontMotor.setInverted(TalonFXInvertType.CounterClockwise);
      rightRearMotor.setInverted(TalonFXInvertType.FollowMaster);
      leftFrontMotor.setInverted(TalonFXInvertType.Clockwise);
      leftRearMotor.setInverted(TalonFXInvertType.FollowMaster);
    } else {
      this.reverse = -1.0;
      rightFrontMotor.setInverted(TalonFXInvertType.Clockwise);
      rightRearMotor.setInverted(TalonFXInvertType.FollowMaster);
      leftFrontMotor.setInverted(TalonFXInvertType.CounterClockwise);
      leftRearMotor.setInverted(TalonFXInvertType.FollowMaster);
    }
  }

  /**
   * Invert joysticks
   * 
   * @param mode true to invert
   */
  public void setJoystickInvert(boolean mode) {
    if (mode) {
      invert = -1;
    } else {
      invert = 1;
    }
  }

  /**
   * Get if joystick is inverted
   * 
   * @return true if inverted
   */
  public boolean getJoystickInvert() {
    return invert == -1;
  }

  public void setLeftVoltage(double voltage) {
    leftFrontMotor.set(ControlMode.PercentOutput, voltage / 12.0);
  }

  public void setRightVoltage(double voltage) {
    rightFrontMotor.set(ControlMode.PercentOutput, voltage / 12.0);
  }

  public void setRightPercent(double power) {
    rightFrontMotor.set(ControlMode.PercentOutput, power);
  }

  public void setLeftPercent(double power) {
    leftFrontMotor.set(ControlMode.PercentOutput, power);
  }

  /**
   * Autonomous driving
   * 
   * @param angle    Desired angle
   * @param distance Desired distance
   */
  public void arcadeDrive(double angle, double distance) {
    double w = (100 - Math.abs(angle * 100)) * (distance) + distance * 100;
    double v = (100 - Math.abs(distance * 100)) * (angle) + angle * 100;
    double leftVoltage = (-(v + w) / 200);
    double rightVoltage = ((v - w) / 200);
    setLeftVoltage(leftVoltage);
    setRightVoltage(rightVoltage);
  }

  /**
   * Get the distance the left part of the robot had been moving
   * 
   * @return The distance in meters
   */
  public double getLeftDistance() {
    return (leftRearMotor.getSelectedSensorPosition() / ChassisConstants.KTICKS_PER_METER) * -1;
  }

  /**
   * Get the distance the right part of the robot had been moving
   * 
   * @return The distance in meters
   */
  public double getRightDistance() {
    return rightRearMotor.getSelectedSensorPosition() / ChassisConstants.KTICKS_PER_METER;
  }

  public double getRightEncoder() {
    return Calculations.toRPMFromEncoderConnectToTalon(
        (rightRearMotor.getSelectedSensorVelocity() + rightFrontMotor.getSelectedSensorVelocity()) / 2,
        RobotConstants.KCTRE_MAG_ENCODER_TPR);
  }

  public double getLeftEncoder() {
    return Calculations.toRPMFromEncoderConnectToTalon(
        (leftRearMotor.getSelectedSensorVelocity() + leftFrontMotor.getSelectedSensorVelocity()) / 2,
        RobotConstants.KCTRE_MAG_ENCODER_TPR);
  }

  public double getLeftVelocity() {
    return leftMPS();
  }

  public double getRightVelocity() {
    return rightMPS();
  }

  private double falconTicksToRPM(double falconTicks) {
    return (falconTicks * 600) / 2048;
  }

  /**
   * Get robot velocity in meter per second (MPS)
   * 
   * @param falconTicks Encoder ticks
   * @return Velocity in m/s
   */
  private double falconTicksToWheelVelocity(double falconTicks) {
    return falconTicksToMeters(falconTicks) * 10;
  }

  /**
   * Get the distance the robot had been moving in meters
   * 
   * @param ticks Encoder ticks
   * @return The distance in meters
   */
  private double falconTicksToMeters(double ticks) {
    return ticks * ChassisConstants.KMETER_PER_TICKS;
  }

  public double rightRPM() {
    return falconTicksToRPM(
        (rightFrontMotor.getSelectedSensorVelocity() + rightRearMotor.getSelectedSensorVelocity()) / 2);
  }

  public double leftRPM() {
    return falconTicksToRPM(
        (leftFrontMotor.getSelectedSensorVelocity() + rightRearMotor.getSelectedSensorVelocity()) / 2);
  }

  public double leftMPS() {
    return falconTicksToWheelVelocity(leftRearMotor.getSelectedSensorVelocity());
  }

  public double rightMPS() {
    return falconTicksToWheelVelocity(rightRearMotor.getSelectedSensorVelocity());
  }

  public void setIdleMode(NeutralMode mode) {
    leftRearMotor.setNeutralMode(mode);
    leftFrontMotor.setNeutralMode(mode);
    rightFrontMotor.setNeutralMode(mode);
    rightRearMotor.setNeutralMode(mode);
  }

  public double getAngle() {
    return (navx.getYaw() * this.reverse);
  }

  public double getRoll() {
    return (navx.getRoll());
  }

  public boolean canOpenPassiveArm() {
    return getRoll() >= 4;
  }

  public void resetSensors() {
    navx.reset();
    leftFrontMotor.setSelectedSensorPosition(0);
    leftRearMotor.setSelectedSensorPosition(0);
    rightFrontMotor.setSelectedSensorPosition(0);
    rightRearMotor.setSelectedSensorPosition(0);

    odometryHandler.reset();
  }

  public void resetPID() {
    leftVelocityPID.reset();
    rightVelocityPID.reset();
    anglePIDVision.reset();
  }

  public double getVisionAnglePIDOutput(double setpoint) {
    return anglePIDVision.calculate(Limelight.getX() * -1, setpoint);
  }

  public boolean isVisionAngleAtSetpoint() {
    return anglePIDVision.atSetpoint();
  }

  public boolean isPIDRightVelocityAtSetPoint() {
    return rightVelocityPID.atSetpoint();
  }

  public boolean isPIDLeftVelocityAtSetPoint() {
    return leftVelocityPID.atSetpoint();
  }

  public void setRightVelocitySetpoint(double setpoint) {
    rightVelocityPID.setSetpoint(setpoint);
  }

  public void setLeftVelocitySetpoint(double setpoint) {
    leftVelocityPID.setSetpoint(setpoint);
  }

  public double rightVelocityMApathPIDOutput() {
    return rightVelocityPID.calculate(getRightVelocity());
  }

  public double leftVelocityMApathPIDOutput() {
    return leftVelocityPID.calculate(getLeftVelocity());
  }

  public double getRightPID(double measurement) {
    return rightVelocityPID.calculate(measurement);
  }

  public double getLeftPID(double measurement) {
    return leftVelocityPID.calculate(measurement);
  }

  public double getLeftF() {
    return (leftVelocityPID.getSetpoint() / ChassisConstants.MAX_VELOCITY_MPS)
        * ChassisConstants.KF_MAPATH_LEFT_VELOCITY;
  }

  public double getRightF() {
    return (rightVelocityPID.getSetpoint() / ChassisConstants.MAX_VELOCITY_MPS)
        * ChassisConstants.KF_MAPATH_RIGHT_VELOCITY;
  }

  public OdometryHandler getOdomoteryHandler() {
    return odometryHandler;
  }

  public int getLeftColorSensor() {
    return colorSensorLeft.getIR();
  }

  public int getRightColorSensor() {
    return colorSensorRight.getIR();
  }

  @Override
  public void periodic() {
    odometryHandler.update();
    chassisShuffleboard.addNum("right distance", getRightDistance());
    chassisShuffleboard.addNum("left distance", getLeftDistance());
    chassisShuffleboard.addNum("Right Velocity", getRightVelocity());
    chassisShuffleboard.addNum("left Velocity", getLeftVelocity());
    chassisShuffleboard.addString("Robot Point", odometryHandler.getCurrentPosition().toString());
    
    chassisShuffleboard.addNum("angle", frc.robot.Limelight.getX());

    chassisShuffleboard.addNum("distance", frc.robot.Limelight.distance());

    chassisShuffleboard.addNum("yaw", navx.getYaw());
    chassisShuffleboard.addNum("roll", navx.getRoll());
    chassisShuffleboard.addNum("pitch", navx.getPitch());

    chassisShuffleboard.addBoolean("tv", Limelight.getTv());

    chassisShuffleboard.addBoolean("Vision Angle At Setpoint", anglePIDVision.atSetpoint());
    chassisShuffleboard.addBoolean("In Shooting Distance", Limelight.getX() < 5);
    chassisShuffleboard.addNum("time", Timer.getFPGATimestamp());

  }
}