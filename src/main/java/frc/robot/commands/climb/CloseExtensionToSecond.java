// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climb.ClimbExtension;

public class CloseExtensionToSecond extends CommandBase {
  /** Creates a new ClimbToSecend. */
 private double deltaCurrent = 0;
 private double lastCurrent = 0;
  public CloseExtensionToSecond() {
     addRequirements(ClimbExtension.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    if(deltaCurrent > 20){
     ClimbExtension.getInstance().setVoltage(ClimbExtension.getInstance().calculate());
    }else{
      ClimbExtension.getInstance().setVoltage(3);
      deltaCurrent = ClimbExtension.getInstance().getCurrent() - lastCurrent;
      lastCurrent = ClimbExtension.getInstance().getCurrent();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ClimbExtension.getInstance().setSetpoint(ClimbExtension.getInstance().getDistance());
    ClimbExtension.getInstance().setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
