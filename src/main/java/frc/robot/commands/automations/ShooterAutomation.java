// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;

import java.util.function.Supplier;

import com.ma5951.utils.Limelight;
import com.ma5951.utils.commands.PistonCommand;
import com.ma5951.utils.motor.Piston;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.pistonCommand2;
import frc.robot.commands.chassis.PIDVision;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterAutomation extends SequentialCommandGroup {
  /** Creates a new ShooterAutomation. */
  
  public ShooterAutomation(boolean hood) {
    // Add your commands in the addCommands() call, e.g.
    addCommands(
      new InstantCommand(()-> Shooter.getinstance().close()),
      new InstantCommand(()-> NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0)),
      new PIDVision(0),
        new ParallelCommandGroup(
          new ShooterCommand(() -> Shooter.getinstance().getShooterPower()),
          new UpperConveyorCommand()
      ),
      new InstantCommand(()-> NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1))
    );
  }
}
