// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.chassis.PIDVision;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.subsystems.shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterAutomationTelop extends SequentialCommandGroup {
  /** Creates a new ShooterAutomationTelop. */
  public ShooterAutomationTelop() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> Shooter.getinstance().close()),
      new PIDVision(0),
      new ParallelCommandGroup(
          new ShooterCommand(() -> Shooter.getinstance().getShooterPower()),
          new UpperConveyorCommand().perpetually())
    );
  }
}
