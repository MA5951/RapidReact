// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.automations.ShootingAutomation;
import frc.robot.commands.conveyor.ConveyBallsCommand;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import com.ma5951.utils.JoystickContainer;
import com.ma5951.utils.commands.MotorCommand;
import com.ma5951.utils.commands.TogglePistonCommand;

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
    JoystickContainer.AButton.whileActiveContinuous(new MotorCommand(Intake.getinstance(), -0.9));
    JoystickContainer.RB.whenPressed(new TogglePistonCommand(Intake.getinstance()));

    // ---------------------------- Conveyor ----------------------------
    JoystickContainer.BButton.whileActiveContinuous(new ConveyBallsCommand());

    // ---------------------------- Shooter ----------------------------
    JoystickContainer.XButton.whileActiveContinuous(new ShooterCommand());
    JoystickContainer.YButton.whileActiveContinuous(new ShootingAutomation());
    JoystickContainer.LB.whenPressed(new TogglePistonCommand(Shooter.getinstance()));

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
