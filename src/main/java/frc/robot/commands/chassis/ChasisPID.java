package frc.robot.commands.chassis;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.ChassisConstants;

import com.ma5951.utils.JoystickContainer;
import com.ma5951.utils.RobotConstants;

public class ChasisPID extends CommandBase {

  public static Joystick leftJoystick = new Joystick(RobotConstants.KLEFT_JOYSTICK_PORT);
  public static Joystick rightJoystick = new Joystick(RobotConstants.KRIGHT_JOYSTICK_PORT);

  private Chassis chassis;

  public ChasisPID() {
    chassis = Chassis.getinstance();
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    chassis.resetPID();

  }

  @Override
  public void execute() {
    if (JoystickContainer.rightJoystick.getRawButton(1)) {
        chassis.setRightVelocitySetpoint(JoystickContainer.rightJoystick.getY() * ChassisConstants.MAX_VELOCITY_MPS * 0.4);
        chassis.setLeftVelocitySetpoint(JoystickContainer.leftJoystick.getY() * ChassisConstants.MAX_VELOCITY_MPS * 0.4);
    } else {
      chassis.setRightVelocitySetpoint(JoystickContainer.rightJoystick.getY() * ChassisConstants.MAX_VELOCITY_MPS);
      chassis.setLeftVelocitySetpoint(JoystickContainer.leftJoystick.getY() * ChassisConstants.MAX_VELOCITY_MPS);
    }
    if (Math.abs(JoystickContainer.rightJoystick.getY()) > 0.1){
      double rightPower = ChassisConstants.KV_MAPATH_RIGHT_VELOCITY + chassis.getRightPID(chassis.getRightVelocity()) + chassis.getRightF();
      chassis.setRightPercent(rightPower * chassis.inverte);
    }else{
      chassis.setRightPercent(0);
    }
    if (Math.abs(JoystickContainer.leftJoystick.getY()) > 0.1){
      double leftPower = ChassisConstants.KV_MAPATH_LEFT_VELOCITY + chassis.getLeftPID(chassis.getLeftVelocity()) + chassis.getLeftF();
      chassis.setLeftPercent(leftPower * chassis.inverte);
    }else{
      chassis.setLeftPercent(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}