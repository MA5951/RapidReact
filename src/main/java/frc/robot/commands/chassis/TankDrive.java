// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import com.ma5951.utils.JoystickContainer;
import com.ma5951.utils.RobotConstants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.ChassisConstants;

public class TankDrive extends CommandBase {
  /** Creates a new TankDrive. */
  public static Joystick leftJoystick = new Joystick(RobotConstants.KLEFT_JOYSTICK_PORT);
  public static Joystick rightJoystick = new Joystick(RobotConstants.KRIGHT_JOYSTICK_PORT);

  private Chassis chassis;

  public TankDrive() {
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
    if (rightJoystick.getRawButton(1)) {
      chassis.setRightPercent(-JoystickContainer.leftJoystick.getY() * 0.4);
      chassis.setLeftPercent(-JoystickContainer.rightJoystick.getY() * 0.4);
    } else {
      chassis.setRightPercent(-JoystickContainer.leftJoystick.getY());
      chassis.setLeftPercent(-JoystickContainer.rightJoystick.getY());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
