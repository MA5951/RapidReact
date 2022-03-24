// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ma5951.utils.JoystickContainer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Automations.DoubleTapShootingAutomation;
import frc.robot.commands.Automations.ShooterAutomation;

public class ShooterTypeSwitch extends CommandBase {
  /** Creates a new ShooterTypeSwitch. */
  private DoubleTapShootingAutomation doubleTapShootingAutomation;
  public ShooterTypeSwitch() {
    doubleTapShootingAutomation = new DoubleTapShootingAutomation();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    doubleTapShootingAutomation.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (JoystickContainer.rightJoystick.getRawAxis(3) < 0){
      doubleTapShootingAutomation.execute();
    }else {
      new ShooterAutomation();
      
    }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    doubleTapShootingAutomation.end(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
