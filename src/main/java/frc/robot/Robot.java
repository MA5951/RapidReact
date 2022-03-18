// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ma5951.utils.JoystickContainer;
import com.ma5951.utils.Limelight;
import com.ma5951.utils.commands.PistonCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.autonomous.AutonomousPaths.GreenPathAutonomous;
import frc.robot.commands.MotorCommandSuplier;
import frc.robot.commands.chassis.ChassisPID;
import frc.robot.commands.chassis.TankDrive;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.climb.ClimbExtension;
import frc.robot.subsystems.climb.ClimbRotation;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.intake.Intake;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  private UsbCamera camera;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    // Shuffleboard.selectTab("Pre-Match");
    Shuffleboard.getTab("Commands").add("Open Intake", new PistonCommand(Intake.getInstance(), true));
    Shuffleboard.getTab("Commands").add("Close Intake", new PistonCommand(Intake.getInstance(), false));
    Shuffleboard.getTab("Commands").add("Shooter Piston Fender", new PistonCommand(Shooter.getInstance(), true));
    Shuffleboard.getTab("Commands").add("Shooter Piston LaunchZone", new PistonCommand(Shooter.getInstance(), false));
    Limelight.pipeline(0);

    camera = CameraServer.startAutomaticCapture();
    camera.setResolution(80, 60);

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    Limelight.periodic();
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    Chassis.getinstance().setIdleMode(NeutralMode.Coast);
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // schedule the autonomous command (example)
    Shooter.getInstance().close();
    Conveyor.getInstance().setAmountOfBalls(1);
    Chassis.getinstance().resetSensors();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    Chassis.getinstance();
    Shooter.getInstance();
    Conveyor.getInstance();
    ClimbExtension.getInstance();
    Conveyor.getInstance().setAmountOfBalls(1);
    // ClimbExtension.getInstance();
    // // ClimbPassive.getInstance();
    // ClimbRotation.getInstance();
    Conveyor.getInstance().setAmountOfBalls(0);
    ClimbRotation.getInstance().reset();
    ClimbExtension.getInstance().reset();

    CommandScheduler.getInstance().setDefaultCommand(Chassis.getinstance(), new TankDrive());
    
    CommandScheduler.getInstance().setDefaultCommand(ClimbExtension.getInstance(), new MotorCommandSuplier(
        ClimbExtension.getInstance(),
        () -> Math.abs(JoystickContainer.operatingJoystick.getRawAxis(1)) > 0.3
            ? JoystickContainer.operatingJoystick.getRawAxis(1) * 0.4
            : 0));
           //  CommandScheduler.getInstance().setDefaultCommand(ClimbExtension.getInstance(), new Extancen());
    CommandScheduler.getInstance().setDefaultCommand(ClimbRotation.getInstance(), new MotorCommandSuplier(
        ClimbRotation.getInstance(),
        () -> Math.abs(JoystickContainer.operatingJoystick.getRawAxis(4)) > 0.3
            ? JoystickContainer.operatingJoystick.getRawAxis(4) * 0.4
            : 0));

    // CommandScheduler.getInstance().setDefaultCommand(ClimbRotation.getInstance(),
    // new ControlCommand(ClimbRotation.getInstance(), 0, false, true));
    Shuffleboard.selectTab("Teleop");
  }
  
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
