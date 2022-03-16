// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.commands.conveyor.ConveyorCommnadAutonomous;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedPathAutonomous extends SequentialCommandGroup {
	/**
	 * Three Balls Autonomous
	 */
	public RedPathAutonomous() {
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		addCommands(
				new ParallelDeadlineGroup(
						new ParallelCommandGroup(
								new WaitCommand(3),
								new AutonomousCommand(Paths.gettingOutOfLunchPad, true),
								new IntakeAutomation(0.8))),
				new PIDVision(Shooter.getInstance().calculateAngle()),
				new ParallelDeadlineGroup(
						new UpperConveyorCommand(),
						new ShooterCommand(() -> Shooter.getInstance().calculateRPM())),
				new AutonomousCommand(Paths.goToTheSecondBallPart1, true),
				new ParallelDeadlineGroup(
						new AutonomousCommand(Paths.goToTheSecondBallPart2, true),
						new IntakeAutomation(0.8)),
				new ParallelDeadlineGroup(
						new AutonomousCommand(Paths.goToTheSecondBallPart3, false),
						new ConveyorCommnadAutonomous(),
						new ShooterCommand(-200)),
				new InstantCommand(() -> Conveyor.getInstance().setAmountOfBalls(1)),
				new ParallelDeadlineGroup(
						new PIDVision(Shooter.getInstance().calculateAngle()),
						new ShooterCommand(-200)),
				new ShooterCommand(() -> Shooter.getInstance().calculateRPM())
						.alongWith(new UpperConveyorCommand()));
	}
}
