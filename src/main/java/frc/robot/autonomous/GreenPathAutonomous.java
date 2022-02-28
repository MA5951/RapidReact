// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package frc.robot.autonomous;
package frc.robot.autonomous;

// import com.ma5951.utils.Limelight;
// import com.ma5951.utils.commands.ControlCommand;


// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.automations.UpperConveyorCommand;
// import frc.robot.commands.chassis.AutonomousCommand;
// import frc.robot.commands.chassis.Paths;
// import frc.robot.commands.shooter.ShooterCommand;
// import frc.robot.subsystems.shooter.Shooter;
// import frc.robot.subsystems.shooter.ShooterConstants;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Automations.IntakeAutomation;
import frc.robot.commands.Automations.UpperConveyorCommand;
import frc.robot.commands.chassis.AutonomousCommand;
import frc.robot.commands.chassis.PIDVision;
import frc.robot.commands.chassis.Paths;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class GreenPathAutonomous extends SequentialCommandGroup {
//   /** Creates a new GreenPathAutonomous. */
//   public GreenPathAutonomous() {
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     addCommands(
//         new AutonomousCommand(Paths.getingOutOfLunchPad),
//         new ParallelCommandGroup(
//           new ControlCommand(Shooter.getinstance(), () -> Shooter.getinstance().getRPMVelocity(), false, true)), 
//           new UpperConveyorCommand()
//     );
//   }
// }
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GreenPathAutonomous extends SequentialCommandGroup {
  /** Creates a new GreenPathAutonomous. */
  public GreenPathAutonomous() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelDeadlineGroup(
            new AutonomousCommand(Paths.getingOutOfLunchPad, true),
            new IntakeAutomation(0.5)
        ),
        new PIDVision(0),
        new ParallelCommandGroup(
          new ShooterCommand(() -> Shooter.getinstance().getShooterPower())).alongWith(
          new UpperConveyorCommand())
    );
  }
}