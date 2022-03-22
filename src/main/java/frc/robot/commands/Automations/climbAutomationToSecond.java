// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;

import com.ma5951.utils.commands.ControlCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climb.ClimbExtension;
import frc.robot.subsystems.intake.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class climbAutomationToSecond extends SequentialCommandGroup {
	/** Second Bar Automation */
	public climbAutomationToSecond() {
		addCommands(
				// new InstantCommand(() -> ClimbRotation.getInstance().setF(2.5)),
				// new InstantCommand(() -> ClimbRotation.getInstance().setIntegratorRange(-0.15, 0.15)),
				new InstantCommand(() -> Intake.getInstance().close()),
				new SequentialCommandGroup(
						new ControlCommand(ClimbExtension.getInstance(), 0, true, true),
						new ControlCommand(ClimbExtension.getInstance(), -0.01, true, true))
		);
						// new ParallelCommandGroup(
						// 		new ControlCommand(ClimbRotation.getInstance(), -12, false, true),
						// 		new ControlCommand(ClimbExtension.getInstance(), -0.02, false, true))));
}
}
