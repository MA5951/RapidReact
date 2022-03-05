// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;

import com.ma5951.utils.commands.ControlCommand;
import com.ma5951.utils.commands.MotorCommand;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.climb.ClimbExtension;
import frc.robot.subsystems.climb.ClimbRotation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbAutomationToThird extends SequentialCommandGroup {
  /** Creates a new ClimbAutomationToThird. */
  public ClimbAutomationToThird() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ControlCommand(ClimbRotation.getInstance(), 3, true, true),
        new ControlCommand(ClimbExtension.getInstance(),
            -125000, true, true),
        // new ParallelDeadlineGroup(
        // new WaitUntilCommand(Chassis.getinstance()::canOpenPassiveArm),
        new MotorCommand(ClimbExtension.getInstance(), 0.4));// );
  }
}
