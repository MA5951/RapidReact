// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MAUtils2.MACommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.MAUtils2.MASubsystem.PistonInterfaceSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MATogglePistonCommand extends InstantCommand {

    private PistonInterfaceSubsystem subsystem;

  public MATogglePistonCommand(PistonInterfaceSubsystem subsystem) {
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (subsystem.isOpen()){
      subsystem.close();
    } else {
      subsystem.open();
    }
  }
}