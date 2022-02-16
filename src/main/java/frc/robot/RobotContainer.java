// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.text.StyledEditorKit.BoldAction;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Automations.IntakeAutomation;
import frc.robot.commands.Automations.IntakeConveyanceAutomation;
import frc.robot.commands.Automations.ShootingAutomation;
import frc.robot.commands.Conveyor.ConveyorCommand;
import frc.robot.commands.Shooter.ShooterCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.JoystickContainer;
import frc.robot.utils.commands.MAMotorCommand;
import frc.robot.utils.commands.MATogglePistonCommand;

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
  JoystickButton controlModeSwitch;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    boolean robotControlMode = true;
    JoystickButton controlModeSwitch = new JoystickButton(JoystickContainer.leftJoystick, 3);
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
    controlModeSwitch.whenPressed(new ControlModeSwitch());
    if (robotControlMode) {// ---------------------------- Auto Mode ----------------------------
      // ---------------------------- Intake ----------------------------
      JoystickContainer.AButton.whileActiveContinuous(new IntakeAutomation(-0.9));
      // ---------------------------- Conveyor ----------------------------
      JoystickContainer.BButton.whileActiveContinuous(new IntakeConveyanceAutomation());
      // ---------------------------- Shooter ----------------------------
      JoystickContainer.YButton.whileActiveContinuous(new ShootingAutomation());
      // ---------------------------- Climb ----------------------------

    } else {// ---------------------------- Manual Mode ----------------------------
      // ---------------------------- Intake ----------------------------
      JoystickContainer.AButton.whileActiveContinuous(new MAMotorCommand(Intake.getinstance(), -0.9));
      JoystickContainer.RB.whenPressed(new MATogglePistonCommand(Intake.getinstance()));

      // ---------------------------- Conveyor ----------------------------
      JoystickContainer.BButton.whileActiveContinuous(new ConveyorCommand());

      // ---------------------------- Shooter ----------------------------
      JoystickContainer.XButton.whileActiveContinuous(new ShooterCommand(3275));
      JoystickContainer.LB.whenPressed(new MATogglePistonCommand(Shooter.getinstance()));

      // ---------------------------- Climb ----------------------------

    }
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
