// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automations;

import com.ma5951.utils.commands.MotorCommand;
import com.ma5951.utils.commands.PistonCommand;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeConveyanceAutomation extends ParallelDeadlineGroup {
  /** Creates a new IntakeConveyanceCommand. */
  public IntakeConveyanceAutomation(double power) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(
        new frc.robot.commands.conveyor.ConveyorCommand(),
        new SequentialCommandGroup(
            new PistonCommand(Intake.getinstance(), true),
            new MotorCommand(Intake.getinstance(), power)));
  }
}
