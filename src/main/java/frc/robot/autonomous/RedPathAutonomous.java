// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import com.ma5951.utils.autonomous.Path;
import com.ma5951.utils.commands.TogglePistonCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ConveyorCommnadAutonomous;
import frc.robot.commands.IntakeCommandForAutonomous;
import frc.robot.commands.Automations.IntakeAutomation;
import frc.robot.commands.Automations.IntakeAutomationAutonomous;
import frc.robot.commands.Automations.UpperConveyorCommand;
import frc.robot.commands.chassis.AutonomousCommand;
import frc.robot.commands.chassis.PIDVision;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.commands.chassis.Paths;
import frc.robot.commands.conveyor.ConveyorCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedPathAutonomous extends SequentialCommandGroup {
  /** Creates a new RedPathAutonomous. */
  public RedPathAutonomous() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new AutonomousCommand(Paths.getingOutOfLunchPadPart1, true),
        new ParallelDeadlineGroup(
            new AutonomousCommand(Paths.getingOutOfLunchPadPart2, true),
            new IntakeAutomation(0.7)),
        new PIDVision(0),
        new ParallelDeadlineGroup(
            new UpperConveyorCommand(),
            new ShooterCommand(() -> Shooter.getinstance().getShooterPower())),
        new AutonomousCommand(Paths.goToTheSecondBallPart1, true),
        new ParallelDeadlineGroup(
            new AutonomousCommand(Paths.goToTheSecondBallPart2, true),
            new IntakeAutomation(0.7)),
        new ParallelDeadlineGroup(
            new AutonomousCommand(Paths.goToTheSecondBallPart3, false),
            new ConveyorCommnadAutonomous()),
        new ShooterCommand(-2400).alongWith(new UpperConveyorCommand()));
  }
}
