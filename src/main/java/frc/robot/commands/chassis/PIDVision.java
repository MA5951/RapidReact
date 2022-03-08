/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.chassis;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.Chassis;

public class PIDVision extends CommandBase {
  /**
   *  Center In Front Of A Target
   */

  private Chassis chassis;
  private double angle;

  public PIDVision(double angle) {
    this.angle = angle;
    chassis = Chassis.getinstance();
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = chassis.getVisionAnglePIDOutput(angle);
    chassis.arcadeDrive(output, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.setLeftVoltage(0);
    chassis.setRightVoltage(0);
    chassis.resetPID();
    chassis.setIdleMode(NeutralMode.Brake);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return chassis.isVisionAngleAtSetpoint();
  }
}