// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;

import com.ma5951.utils.commands.PistonCommand;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCommandForAutonomous;
import frc.robot.commands.conveyor.ConveyorCommand;
import frc.robot.subsystems.intake.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAutomationAutonomous extends SequentialCommandGroup {
  /** Creates a new IntakeAutomationAutonomous. */
  public IntakeAutomationAutonomous(double power) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new PistonCommand(Intake.getinstance(), true),
        new ParallelDeadlineGroup(
            new IntakeCommandForAutonomous(power),
            new ConveyorCommand()));
  }
}
