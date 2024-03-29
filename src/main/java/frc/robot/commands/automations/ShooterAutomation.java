// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;

import com.ma5951.utils.JoystickContainer;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.chassis.PIDVision;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.subsystems.shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterAutomation extends SequentialCommandGroup {
  /** Center In Front Of The Hub And Shoot Ball Automation */

  public ShooterAutomation() {
    // Add your commands in the addCommands() call, e.g.
    addCommands(
        new PIDVision(Shooter.getInstance().calculateAngle()),
        new ParallelCommandGroup(
            new ShooterCommand(() -> Shooter.getInstance().calculateRPM()),
            new UpperConveyorCommand()));
  }
}
