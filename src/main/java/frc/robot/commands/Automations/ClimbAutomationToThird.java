// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;

import com.ma5951.utils.commands.ControlCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climb.ClimbExtension;
import frc.robot.subsystems.climb.ClimbRotation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbAutomationToThird extends SequentialCommandGroup {
  /** Third Bar Automation */
  public ClimbAutomationToThird() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ControlCommand(ClimbExtension.getInstance(),
            0.1, true, true),
        new InstantCommand(() -> ClimbRotation.getInstance().setF(0)),
        new InstantCommand(() -> ClimbRotation.getInstance().setIntegratorRange(-0.1, 0.1)),
        new InstantCommand(() -> ClimbRotation.getInstance().setOutputRange(-2, 2)),
        new ControlCommand(ClimbRotation.getInstance(), -28, true, true),
        new ParallelCommandGroup(
            new ControlCommand(ClimbExtension.getInstance(),
                0.86, false, true),
            new ControlCommand(ClimbRotation.getInstance(), -28, false, true)));
  }
}
