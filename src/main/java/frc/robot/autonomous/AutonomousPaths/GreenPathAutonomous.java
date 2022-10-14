// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package frc.robot.autonomous;
package frc.robot.autonomous.AutonomousPaths;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autonomous.Paths;
import frc.robot.commands.Automations.IntakeAutomation;
import frc.robot.commands.Automations.UpperConveyorCommand;
import frc.robot.commands.chassis.AutonomousCommand;
import frc.robot.commands.chassis.PIDVision;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GreenPathAutonomous extends SequentialCommandGroup {
    /**
     * Two Balls Autonomous
     */
    public GreenPathAutonomous() {
        addCommands(
                //new WaitCommand(2),
                new InstantCommand(() -> Intake.getInstance().open()),
                new ParallelDeadlineGroup(
                        new WaitCommand(1.7),
                        new AutonomousCommand(Paths.gettingOutOfLunchPadPart1, false),
                        new InstantCommand(() -> Intake.getInstance().setPower(0.7))
                ),
                // TODO: Replace deadline with race
                new InstantCommand(() -> Intake.getInstance().setPower(0)),
                new PIDVision(Shooter.getInstance().calculateAngle()),
                new ParallelDeadlineGroup(
                        new WaitCommand(3),
                        new ShooterCommand(() -> Shooter.getInstance().calculateRPM()),
                        new UpperConveyorCommand()
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(2),
                        new IntakeAutomation(1)
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(5),
                        new ShooterCommand(() -> Shooter.getInstance().calculateRPM()),
                        new UpperConveyorCommand()
                )
        );
    }
}