// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automations;

import com.ma5951.utils.commands.PistonCommand;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterAutomation extends SequentialCommandGroup {
  /** Creates a new ShooterAutomation. */

  private static final double DISTANCE_THRESHOLD = 100; // TODO

  public ShooterAutomation(double distance, double setpoint) {
    // Add your commands in the addCommands() call, e.g.
    addCommands(
      new PistonCommand(Shooter.getinstance(), () -> distance < DISTANCE_THRESHOLD),
      new ShooterCommand(setpoint, false),
      new ParallelDeadlineGroup(
        new UpperConveyorCommand(),
        new ShooterCommand(setpoint)
      )
    );
  }
}
