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

    if (JoystickContainer.rightJoystick.getY() > 0.1 || JoystickContainer.rightJoystick.getY() < -0.1) {
      if (JoystickContainer.rightJoystick.getRawButtonPressed(1)){
        //chassis.setRightVelocitySetpoint(JoystickContainer.rightJoystick.getY() * 3.5 * ChassisConstants.KSCALE);
        chassis.setRightPercent(JoystickContainer.rightJoystick.getY());
      } else {
        //chassis.setRightVelocitySetpoint(JoystickContainer.rightJoystick.getY() * 3.5);
        chassis.setRightPercent(JoystickContainer.rightJoystick.getY() * ChassisConstants.KSCALE);
      }
    } else {
      chassis.setRightVelocitySetpoint(0);
    }

    if (JoystickContainer.leftJoystick.getY() > 0.1 || JoystickContainer.leftJoystick.getY() < -0.1) {
      if (JoystickContainer.leftJoystick.getRawButtonPressed(1)){
        //chassis.setLeftVelocitySetpoint(JoystickContainer.leftJoystick.getY() * ChassisConstants.KSCALE * 3.5);
        chassis.setLeftPercent(JoystickContainer.leftJoystick.getY() * ChassisConstants.KSCALE);
      } else {
        //chassis.setLeftVelocitySetpoint(JoystickContainer.leftJoystick.getY() * 3.5);
        chassis.setLeftPercent(JoystickContainer.leftJoystick.getY());
      }
    } else {
     //chassis.setLeftVelocitySetpoint(0);
     chassis.setLeftPercent(0);
    }

    // chassis.setLeftPercent(chassis.leftVelocityMApathPIDOutput());
    // chassis.setRightPercent(chassis.rightVelocityMApathPIDOutput());
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