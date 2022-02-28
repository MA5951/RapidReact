package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.ChassisConstants;

import com.ma5951.utils.JoystickContainer;

public class TankDrive extends CommandBase {
  private Chassis chassis;

  public TankDrive() {
    chassis = Chassis.getinstance();
    addRequirements(chassis);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

    if (JoystickContainer.rightJoystick.getY() > 0.1 ||
    JoystickContainer.rightJoystick.getY() < -0.1) {
    if (JoystickContainer.rightJoystick.getRawButtonPressed(1)){
    chassis.setRightPercent(JoystickContainer.rightJoystick.getY() * ChassisConstants.KSCALE);
    } else {
    chassis.setRightPercent(JoystickContainer.rightJoystick.getY());
    }
    } else {
    chassis.setRightPercent(0);
    }

    if (JoystickContainer.leftJoystick.getY() > 0.1 ||
    JoystickContainer.leftJoystick.getY() < -0.1) {
    if (JoystickContainer.rightJoystick.getRawButtonPressed(1)){
    chassis.setLeftPercent(JoystickContainer.leftJoystick.getY() *
    ChassisConstants.KSCALE);
    } else {
 
    chassis.setLeftPercent(JoystickContainer.leftJoystick.getY());
    }
    } else {
    chassis.setLeftPercent(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    chassis.setLeftVoltage(0);
    chassis.setRightVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}