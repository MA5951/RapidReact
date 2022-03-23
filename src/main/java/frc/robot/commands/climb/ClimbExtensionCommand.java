// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import com.ma5951.utils.JoystickContainer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.climb.ClimbExtension;

public class ClimbExtensionCommand extends CommandBase {
  /** Creates a new ClimbExtensionCommand. */
  private ClimbExtension climbExtension;
  public ClimbExtensionCommand() {
    climbExtension = ClimbExtension.getInstance();
    addRequirements(climbExtension);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climbExtension.getDistance() / ClimbConstants.TICK_PER_METER_EXTENSION <= 0.6 && (JoystickContainer.operatingJoystick.getLeftY() < -0.3)){
          climbExtension.setPower(-0.6);
    }else if (JoystickContainer.operatingJoystick.getLeftY() > 0.3){
          climbExtension.setPower(0.6);
    }
    else {
      climbExtension.setPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbExtension.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
