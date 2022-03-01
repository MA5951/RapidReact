package frc.robot.subsystems.chassis;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
  private PIDController distancePIDVision;

  public Shuffleboard chassisShuffleboard;

  private OdometryHandler odometryHandler;

  private double revarse = 1;

  private ColorSensorV3 colorSensorLeft;
  private ColorSensorV3 colorSensorRight;

  private boolean inverted;

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
    distancePIDVision = new PIDController(ChassisConstants.KP_VISION_DISTANCE, ChassisConstants.KI_VISION_DISTANCE,
        ChassisConstants.KD_VISION_DISTANCE, 0, 2, -12, 12);

    anglePIDVision.enableContinuousInput(-ChassisConstants.KANGLE_PID_VISION_SET_INPUTRANGE,
        ChassisConstants.KANGLE_PID_VISION_SET_INPUTRANGE);

    colorSensorLeft = new ColorSensorV3(I2C.Port.kMXP);
    colorSensorRight = new ColorSensorV3(I2C.Port.kOnboard);

    resetSensors();
    rightFrontMotor.setInverted(TalonFXInvertType.Clockwise);
    rightRearMotor.setInverted(TalonFXInvertType.FollowMaster);

  }

  public void setInverted(boolean revarse){
    if (revarse){
      this.revarse = 1.0;
      rightFrontMotor.setInverted(TalonFXInvertType.CounterClockwise);
      rightRearMotor.setInverted(TalonFXInvertType.FollowMaster);
      leftFrontMotor.setInverted(TalonFXInvertType.Clockwise);
      leftRearMotor.setInverted(TalonFXInvertType.FollowMaster);
    } else {
      this.revarse = -1.0;
      rightFrontMotor.setInverted(TalonFXInvertType.Clockwise);
      rightRearMotor.setInverted(TalonFXInvertType.FollowMaster);
      leftFrontMotor.setInverted(TalonFXInvertType.CounterClockwise);
      leftRearMotor.setInverted(TalonFXInvertType.FollowMaster);
    }
    inverted = revarse;

  }

  public boolean getInverted(){
    return inverted;
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

  public void arcadeDrive(double angle, double distance) {
    double w = (100 - Math.abs(angle * 100)) * (distance) + distance * 100;
    double v = (100 - Math.abs(distance * 100)) * (angle) + angle * 100;
    double leftVoltage = (-(v + w) / 200);
    // System.out.println("Left Voltage" + leftVoltage);
    double rightVoltage = ((v - w) / 200);
    // System.out.println("Right Voltage" + rightVoltage);
    setLeftVoltage(leftVoltage);
    setRightVoltage(rightVoltage);
  }

  public double getLeftDistance() {
    return leftRearMotor.getSelectedSensorPosition() / ChassisConstants.KTICKS_PER_METER;
  }

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

  private double falconTicksToWheelVelocity(double falconTicks) {
    return falconTicksToMeters(falconTicks);
  }

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
    // System.out.println("LEFT " +
    // falconTicksToWheelVelocity(leftRearMotor.getSelectedSensorVelocity()));
    return falconTicksToWheelVelocity(leftRearMotor.getSelectedSensorVelocity());
  }

  public double rightMPS() {
    // System.out.println("RIGHT " +
    // falconTicksToWheelVelocity(rightRearMotor.getSelectedSensorVelocity()));
    return falconTicksToWheelVelocity(rightRearMotor.getSelectedSensorVelocity());
  }

  public void setIdleMode(NeutralMode mode) {
    leftRearMotor.setNeutralMode(mode);
    leftFrontMotor.setNeutralMode(mode);
    rightFrontMotor.setNeutralMode(mode);
    rightRearMotor.setNeutralMode(mode);
  }

  public double getAngle() {
    return (navx.getYaw() * this.revarse);
  }

  // reset the value of the encoder and the navx
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
    distancePIDVision.reset();
    anglePIDVision.reset();
  }

  public double getVisionAnglePIDOutput(double setpoint) {
    return anglePIDVision.calculate(Limelight.x * -1, setpoint);
  }

  public double getVisionDistancePIDOutput(double setpoint) {
    return distancePIDVision.calculate(Limelight.Tshort, setpoint);
  }

  public boolean isVisionAngleAtSetpoint() {
    return anglePIDVision.atSetpoint();
  }

  public boolean isVisionDistanceAtSetpoint() {
    return distancePIDVision.atSetpoint();
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
    return (leftVelocityPID.getSetpoint() / ChassisConstants.MAX_VELOCITY_MPS) * ChassisConstants.KF_MAPATH_LEFT_VELOCITY;
  }

  public double getRightF() {
    return (rightVelocityPID.getSetpoint()/ChassisConstants.MAX_VELOCITY_MPS) * ChassisConstants.KF_MAPATH_RIGHT_VELOCITY;
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
    chassisShuffleboard.addString("Robot Point", odometryHandler.getCurrentPosition().toString());

    // chassisShuffleboard.addNum("leftColorSensor", getLeftColorSensor());
    // chassisShuffleboard.addNum("rightColorSensor", getRightColorSensor());

    chassisShuffleboard.addNum("distance", frc.robot.Limelight.distance());
    //chassisShuffleboard.addNum("Yaw", getAngle());
    chassisShuffleboard.addNum("y", navx.getYaw());
    chassisShuffleboard.addNum("z", navx.getRoll());
    chassisShuffleboard.addNum("y", navx.getPitch());

  }
}