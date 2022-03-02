// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Automations.IntakeAutomation;
import frc.robot.commands.Automations.ShooterAutomation;
import frc.robot.commands.Automations.UpperConveyorCommand;
import frc.robot.commands.chassis.AutonomousCommand;
import frc.robot.commands.chassis.PIDVision;
import frc.robot.commands.chassis.Paths;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OrangePathAutonomous extends SequentialCommandGroup {
  /** Creates a new OrangePathAutonomous. */
  public OrangePathAutonomous() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutonomousCommand(Paths.goToCloseBall, true),
      new IntakeAutomation(0.5),
      new PIDVision(0),
      new ParallelDeadlineGroup(
        new ShooterAutomation(true)
      , new UpperConveyorCommand()),
      new AutonomousCommand(Paths.goToStrightBall, true),
      new IntakeAutomation(0.5),
      new AutonomousCommand(Paths.goToHPBall, true),
      new IntakeAutomation(0.5),
      new AutonomousCommand(Paths.goToShootingPosition, false),
      new PIDVision(0),
      new ParallelDeadlineGroup(
        new ShooterAutomation(true), 
        new UpperConveyorCommand())
    );
  }
}
