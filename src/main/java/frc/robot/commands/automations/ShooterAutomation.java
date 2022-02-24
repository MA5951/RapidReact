// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automations;

import java.util.function.Supplier;

import com.ma5951.utils.commands.PistonCommand;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterAutomation extends SequentialCommandGroup {
  /** Creates a new ShooterAutomation. */
  
  public ShooterAutomation(Supplier<Boolean> hood, Supplier<Double> setpoint) {
    // Add your commands in the addCommands() call, e.g.
    addCommands(
      new PistonCommand(Shooter.getinstance(), hood),
      new ShooterCommand(setpoint, false),
      new ParallelDeadlineGroup(
        new UpperConveyorCommand(),
        new PerpetualCommand(new ShooterCommand(setpoint))
      )
    );
  }
}
