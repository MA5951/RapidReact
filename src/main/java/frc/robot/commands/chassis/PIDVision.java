/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.chassis;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ma5951.utils.JoystickContainer;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LEDManager;
import frc.robot.Limelight;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.conveyor.Conveyor;

public class PIDVision extends CommandBase {
  /**
   * Center In Front Of A Target
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
    Conveyor.getInstance().isInControlLED = false;
    if (Limelight.getX() < 0) {
      this.angle *= -1;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Limelight.getTs() > -4) {
      LEDManager.getInstance().setWhite();
      double output = chassis.getVisionAnglePIDOutput(angle);
      chassis.arcadeDrive(output, 0);
    } else {
      LEDManager.getInstance().setBlue();
    }
    if (Limelight.distance() > 9){
      LEDManager.getInstance().setPurple();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.setLeftVoltage(0);
    chassis.setRightVoltage(0);
    chassis.resetPID();
    Conveyor.getInstance().isInControlLED = true;
    chassis.setIdleMode(NeutralMode.Brake);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return chassis.isVisionAngleAtSetpoint() || !Limelight.getTv();
  }
}