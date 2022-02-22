// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

public class SwitchBasedOnControlMode extends CommandBase {
  /** Creates a new SwitchBasedOnControlMode. */
  public SwitchBasedOnControlMode(Command runOnTrue, Command runOnFalse) {
    new ConditionalCommand(runOnTrue, runOnFalse, () -> RobotContainer.robotControlMode);
  }
}
