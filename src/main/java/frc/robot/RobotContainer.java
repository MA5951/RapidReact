// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.automations.IntakeAutomation;
import frc.robot.commands.automations.ShooterAutomation;
import frc.robot.commands.automations.UpperConveyorCommand;
import frc.robot.commands.conveyor.ConveyBallsCommand;
import frc.robot.commands.conveyor.ConveyorCommand;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.subsystems.climb.ClimbExtension;
import frc.robot.subsystems.climb.ClimbPassive;
import frc.robot.subsystems.climb.ClimbRotation;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

import com.ma5951.utils.JoystickContainer;
import com.ma5951.utils.commands.ControlCommand;
import com.ma5951.utils.commands.MotorCommand;
import com.ma5951.utils.commands.PistonCommand;
import com.ma5951.utils.commands.RunCommand;
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
    //controlModeToogle.whenPressed(new InstantCommand(RobotContainer::toogleControlMode));

    // // ---------------------------- Intake ----------------------------
    // JoystickContainer.AButton.whileActiveContinuous(new MotorCommand(Intake.getinstance(), -0.9)); // X button
    JoystickContainer.BButton.whenPressed(new TogglePistonCommand(Intake.getinstance()));

        //.whenReleased(Intake.getinstance()::off);
    // JoystickContainer.AButton.whileActiveContinuous(new IntakeAutomation(.9)); // X button
    JoystickContainer.AButton.whileActiveContinuous(new PistonCommand(Intake.getinstance(), true).
    andThen( new MotorCommand(Intake.getinstance(), 0.7).
      alongWith(new ConveyorCommand()))
      ); // X button
    // JoystickContainer.YButton.whenPressed(() -> Conveyor.getInstance().setLowerPower(1))
    //                           .whenReleased(() -> Conveyor.getInstance().setLowerPower(0));

    // JoystickContainer.YButton.whenPressed(() -> Conveyor.getInstance().setUpperPower(-1))
    //                           .whenReleased(() -> Conveyor.getInstance().setUpperPower(0));

    // ---------------------------- Conveyor ----------------------------
    // JoystickContainer.BButton.whileActiveContinuous(new ConveyBallsCommand());
    // JoystickContainer.BButton.whenPressed(() -> Conveyor.getInstance().setLowerPower(0.3)) // A button
    //     .whenReleased(() -> Conveyor.getInstance().setLowerPower(0));
    // JoystickContainer.YButton.whenPressed(() -> Conveyor.getInstance().setUpperPower(-0.3)) // Y button
    //     .whenReleased(() -> Conveyor.getInstance().setUpperPower(0));
    //JoystickContainer.BButton.whileActiveContinuous(new ConveyBallsCommand());

    // ---------------------------- Shooter ----------------------------
    JoystickContainer.YButton.whileActiveContinuous(
        new ShooterCommand(ShooterConstants.SHOOTER_VELOCITY_LAUNCH_PAD)); // B
    //                                                                                                            // button    // JoystickContainer.YButton.whileActiveContinuous(new ShootingAutomation());
    JoystickContainer.POVDown.whenPressed(new TogglePistonCommand(Shooter.getinstance()));
    // JoystickContainer.XButton.whileActiveContinuous( 
    //   new ShooterAutomation(100, ShooterConstants.SHOOTER_VELOCITY_LAUNCH_PAD)
    // );
    // JoystickContainer.XButton.whileActiveContinuous( 
    // new PistonCommand(Shooter.getinstance(), 100 < 100).alongWith(
    //   new ShooterCommand(ShooterConstants.SHOOTER_VELOCITY_LAUNCH_PAD, false)).alongWith(
    //   new ParallelDeadlineGroup(
    //     new UpperConveyorCommand(),
    //     new ShooterCommand(ShooterConstants.SHOOTER_VELOCITY_LAUNCH_PAD)
    //   )));

    JoystickContainer.XButton.whileActiveContinuous(
      //new PistonCommand(Shooter.getinstance(), () -> 150 < 100).alongWith(
      new ShooterCommand(-1000)//Shooter.getinstance()::getShooterPower)
      .alongWith(new UpperConveyorCommand()));
    // JoystickContainer.XButton.whileActiveContinuous(
    //   new ShooterAutomation(() -> true, () -> ShooterConstants.SHOOTER_VELOCITY_LAUNCH_PAD)
    //   ).whenInactive(() -> Shooter.getinstance().setPower(0));
    // ---------------------------- Climb ----------------------------
    // JoystickContainer.POVUp.whileActiveContinuous(new
    // MotorCommand(.getInstance(), 0.1));
    // JoystickContainer.POVDown.whileActiveContinuous(new
    // MotorCommand(ClimbExtension.getInstance(), 0.1));

    // JoystickContainer.POVUp.whileActiveContinuous(new MotorCommand(ClimbExtension.getInstance(), -0.3));
    // JoystickContainer.POVDown.whileActiveContinuous(new MotorCommand(ClimbExtension.getInstance(), 0.3));
    // JoystickContainer.POVLeft.whileActiveContinuous(new MotorCommand(ClimbRotation.getInstance(), 0.22));
    // JoystickContainer.POVRight.whileActiveContinuous(new MotorCommand(ClimbRotation.getInstance(), -0.22));

    // JoystickContainer.LB.whileActiveContinuous(new MotorCommand(ClimbPassive.getInstance(), -0.1));
    // JoystickContainer.RB.whileActiveContinuous(new MotorCommand(ClimbPassive.getInstance(), 0.1));

    // //JoystickContainer.POVLeft.whileActiveContinuous(new MotorCommand(ClimbPassive.getInstance(), 0.1));

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
