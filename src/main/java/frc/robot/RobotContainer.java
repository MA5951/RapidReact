// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MAUtils2.JoystickContainer;
import frc.robot.MAUtils2.RobotConstants;
import frc.robot.MAUtils2.MACommands.MAMotorCommand;
import frc.robot.MAUtils2.MACommands.MATogglePistonCommand;
import frc.robot.MAUtils2.MACommands.MApistonCommand;
import frc.robot.commands.Automations.ShootingAutomation;
import frc.robot.commands.Conveyor.ConveyorCommand;
import frc.robot.commands.Shooter.ShooterCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // ---------------------------- Intake ----------------------------
    JoystickContainer.AButton.whileActiveContinuous(new MAMotorCommand(Intake.getinstance(), -0.9));
    JoystickContainer.RB.whenPressed(new MATogglePistonCommand(Intake.getinstance()));

    // ---------------------------- Conveyor ----------------------------
    JoystickContainer.BButton.whileActiveContinuous(new ConveyorCommand());

    // ---------------------------- Shooter ----------------------------
    JoystickContainer.XButton.whileActiveContinuous(new ShooterCommand());
    JoystickContainer.YButton.whileActiveContinuous(new ShootingAutomation());
    JoystickContainer.LB.whenPressed(new MATogglePistonCommand(Shooter.getinstance()));

    // ---------------------------- Climb ----------------------------
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
