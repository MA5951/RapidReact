// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.Conveyor.ConveyorCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.utils.commands.MAMotorCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeConveyanceCommand extends ParallelDeadlineGroup {
  /** Creates a new IntakeConveyanceCommand. */
  public IntakeConveyanceCommand() {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new ConveyorCommand());
    addCommands(new MAMotorCommand(Intake.getinstance(), -0.9));
    // addCommands(new FooCommand(), new BarCommand());
  }
}
