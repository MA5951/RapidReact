// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.subsystem.ControlInterfaceSubsystem;

public class MAControlCommand extends CommandBase {
  /** Creates a new MAControlCommand. */

  private ControlInterfaceSubsystem subsystem;
  private double setpoint;
  private boolean stoppable;
  private boolean voltage;


  public MAControlCommand(ControlInterfaceSubsystem subsystem, double setpoint, boolean stoppable, boolean voltage) {
    this.subsystem = subsystem;
    this.setpoint = setpoint;
    this.stoppable = stoppable;
    this.voltage = voltage;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.setSetpoint(this.setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (voltage) {
      subsystem.setVoltage(subsystem.calculate());
    } else {
      subsystem.setPower(subsystem.calculate());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (stoppable) {
      if (subsystem.atSetpoint()) {
        return true;
      }
    }
    return false;
  }
}