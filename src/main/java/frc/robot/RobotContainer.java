// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Automations.IntakeAutomation;
import frc.robot.commands.Automations.ShooterAutomation;
import frc.robot.commands.Automations.UpperConveyorCommand;
import frc.robot.commands.Automations.climbAutomation;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.climb.ClimbExtension;
import frc.robot.subsystems.climb.ClimbPassive;
import frc.robot.subsystems.climb.ClimbRotation;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

import java.util.function.BooleanSupplier;

import com.ma5951.utils.JoystickContainer;
import com.ma5951.utils.RobotConstants;
import com.ma5951.utils.commands.ControlCommand;
import com.ma5951.utils.commands.MotorCommand;
import com.ma5951.utils.commands.PistonCommand;
import com.ma5951.utils.commands.TogglePistonCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static boolean robotControlMode;
  public static Joystick leftJoystick = new Joystick(RobotConstants.KLEFT_JOYSTICK_PORT);
  public static Joystick righJoystick = new Joystick(RobotConstants.KRIGHT_JOYSTICK_PORT);
  public static Trigger leftTrigger = new Trigger(
      () -> JoystickContainer.operatingJoystick.getRawAxis(RobotConstants.L_TRIGER) > 0.5);
  public static Trigger rightTrigger = new Trigger(
      () -> JoystickContainer.operatingJoystick.getRawAxis(RobotConstants.R_TRIGER) > 0.5);
  JoystickButton controlModeToogle;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    boolean robotControlMode = true;
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton controlModeToogle = new JoystickButton(JoystickContainer.leftJoystick, 3);

    // // ---------------------------- Intake ----------------------------
    JoystickContainer.AButton.whileActiveContinuous(new IntakeAutomation(0.8));
    new Trigger(() -> JoystickContainer.leftJoystick.getRawButton(1))
        .whenActive(new TogglePistonCommand(Intake.getinstance()));
    new Trigger(() -> JoystickContainer.leftJoystick.getRawButton(8))
        .whenActive(() -> Chassis.getinstance().setJoystickInverte(!Chassis.getinstance().getJoystickInverte()));
    new Trigger(() -> JoystickContainer.rightJoystick.getRawButton(8))
        .whenActive(() -> Chassis.getinstance().setInverted(!Chassis.getinstance().getJoystickInverte()));

    // JoystickContainer.YButton.whileActiveContinuous(new ParallelCommandGroup(
    // new MotorCommand(Intake.getinstance(), -0.8),
    // new CovenyorOutCommand()
    // ));

    // ---------------------------- Conveyor ----------------------------

    // ---------------------------- Shooter ------------ ----------------
    JoystickContainer.XButton.whileActiveContinuous(
        new PistonCommand(Shooter.getinstance(), true).andThen(
            new ShooterCommand(ShooterConstants.SHOOTER_VELOCITY_FENDER).alongWith(new UpperConveyorCommand())));

    JoystickContainer.BButton.whileActiveContinuous(new ShooterAutomation(true));

    JoystickContainer.YButton.whenPressed(new PistonCommand(Shooter.getinstance(), false));
    // ---------------------------- Climb ----------------------------

    JoystickContainer.POVUp
        .whileActiveContinuous(new ControlCommand(ClimbExtension.getInstance(), -130000, true, true));
    JoystickContainer.POVDown.whileActiveContinuous(new climbAutomation());
    JoystickContainer.POVLeft.whileActiveContinuous(new MotorCommand(ClimbRotation.getInstance(), 0.22));
    JoystickContainer.POVRight.whileActiveContinuous(new MotorCommand(ClimbRotation.getInstance(), -0.22));

    JoystickContainer.LB.whileActiveContinuous(new MotorCommand(ClimbPassive.getInstance(), -0.1));
    JoystickContainer.RB.whileActiveContinuous(new MotorCommand(ClimbPassive.getInstance(), 0.1));

    JoystickContainer.startButton
        .whenPressed(() -> Conveyor.getInstance().setAmountOfBalls(Conveyor.getInstance().getAmountOfBalls() + 1));
    JoystickContainer.backButton
        .whenPressed(() -> Conveyor.getInstance().setAmountOfBalls(Conveyor.getInstance().getAmountOfBalls() - 1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
