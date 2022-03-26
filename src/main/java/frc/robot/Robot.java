// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ma5951.utils.JoystickContainer;
import com.ma5951.utils.Limelight;
import com.ma5951.utils.commands.PistonCommand;
import com.ma5951.utils.commands.RunCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.autonomous.AutonomousPaths.BluePathAutonomous;
import frc.robot.autonomous.AutonomousPaths.GreenPathAutonomous;
import frc.robot.autonomous.AutonomousPaths.OrangePathAutonomous;
import frc.robot.autonomous.AutonomousPaths.PurplePathAutonomous;
import frc.robot.autonomous.AutonomousPaths.RedPathAutonomous;
import frc.robot.commands.MotorCommandSuplier;
import frc.robot.commands.chassis.ChassisPID;
import frc.robot.commands.chassis.TankDrive;
import frc.robot.commands.climb.ClimbExtensionCommand;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.climb.ClimbExtension;
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

  private SendableChooser<Command> autoChooser;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    autoChooser = new SendableChooser<Command>();

    autoChooser.setDefaultOption("1 Ball", new BluePathAutonomous());
    autoChooser.addOption("2 Balls", new GreenPathAutonomous());
    autoChooser.addOption("3 Balls", new RedPathAutonomous());
    autoChooser.addOption("3-4 Balls - HP", new OrangePathAutonomous());
    autoChooser.addOption("3 Balls - Finish HP", new PurplePathAutonomous());

    m_robotContainer = new RobotContainer();
    Shuffleboard.getTab("Pre-Match").add("Autonoumus Chooser", autoChooser)
        .withPosition(3, 1).withSize(2, 2);
    Shuffleboard.selectTab("Pre-Match");
    Shuffleboard.getTab("Commands").add("Open Intake", new PistonCommand(Intake.getInstance(), true));
    Shuffleboard.getTab("Commands").add("Close Intake", new PistonCommand(Intake.getInstance(), false));
    Shuffleboard.getTab("Commands").add("Shooter Piston Fender", new PistonCommand(Shooter.getInstance(), true));
    Shuffleboard.getTab("Commands").add("Shooter Piston LaunchZone", new PistonCommand(Shooter.getInstance(), false));
    Limelight.pipeline(0);

    // camera = CameraServer.startAutomaticCapture();
    // camera.setResolution(80, 60);

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
    LEDManager.getInstance().periodic();
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
    Conveyor.getInstance().isInAutonomous = true;
    Chassis.getinstance().resetSensors();
    if (autoChooser.getSelected() != null) {
      autoChooser.getSelected().schedule();
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
    Conveyor.getInstance().isInAutonomous = false;
    // ClimbExtension.getInstance();
    // // ClimbPassive.getInstance();
    // ClimbRotation.getInstance();
    Conveyor.getInstance().setAmountOfBalls(0);
    ClimbExtension.getInstance().reset();

    CommandScheduler.getInstance().setDefaultCommand(Chassis.getinstance(), new TankDrive());

    // CommandScheduler.getInstance().setDefaultCommand(ClimbExtension.getInstance(),
    // new MotorCommandSuplier(
    // ClimbExtension.getInstance(),
    // () -> Math.abs(JoystickContainer.operatingJoystick.getRawAxis(1)) > 0.3
    // ? JoystickContainer.operatingJoystick.getRawAxis(1)
    // : 0));

    CommandScheduler.getInstance().setDefaultCommand(ClimbExtension.getInstance(), new ClimbExtensionCommand());

    CommandScheduler.getInstance().setDefaultCommand(Intake.getInstance(),
        new SequentialCommandGroup(new WaitCommand(1), new RunCommand(Intake.getInstance()::off, () -> {
        }, Intake.getInstance())));

    CommandScheduler.getInstance().setDefaultCommand(Shooter.getInstance(),
        new SequentialCommandGroup(new WaitCommand(1), new RunCommand(Shooter.getInstance()::off, () -> {
        }, Shooter.getInstance())));
    // CommandScheduler.getInstance().setDefaultCommand(ClimbRotation.getInstance(),
    // new ControlCommand(ClimbRotation.getInstance(), 0, false, true));
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
