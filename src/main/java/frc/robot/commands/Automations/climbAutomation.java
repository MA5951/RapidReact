// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;

import com.ma5951.utils.commands.ControlCommand;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClimbPassiveCommand;
import frc.robot.subsystems.climb.ClimbExtension;
import frc.robot.subsystems.climb.ClimbRotation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class climbAutomation extends SequentialCommandGroup {
  /** Creates a new climbAutomation. */
  public climbAutomation() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ControlCommand(ClimbExtension.getInstance(), -130000, true, true),
      //new ControlCommand(ClimbRotation.getInstance(), -1, true, true),
        new ControlCommand(ClimbExtension.getInstance(), 0, true, true).alongWith(
        new ControlCommand(ClimbRotation.getInstance(), -2, true, true)),
      new ClimbPassiveCommand(0.1)
    );
  }
}
