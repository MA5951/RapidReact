package frc.robot.commands.chassis;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.ChassisConstants;

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
    if (rightJoystick.getRawButton(1) || leftJoystick.getRawButton(1)) {
      chassis.setRightVelocitySetpoint(-rightJoystick.getY() * ChassisConstants.MAX_VELOCITY_MPS * 0.4);
      chassis.setLeftVelocitySetpoint(leftJoystick.getY() * ChassisConstants.MAX_VELOCITY_MPS * 0.4);
    } else {
      chassis.setRightVelocitySetpoint(-rightJoystick.getY() * ChassisConstants.MAX_VELOCITY_MPS);
      chassis.setLeftVelocitySetpoint(leftJoystick.getY() * ChassisConstants.MAX_VELOCITY_MPS);
    }

    chassis.setRightPercent(ChassisConstants.KV_MAPATH_RIGHT_VELOCITY + 
      chassis.getRightPID(chassis.getRightVelocity()) + chassis.getRightF());
    chassis.setLeftPercent(ChassisConstants.KV_MAPATH_LEFT_VELOCITY + 
      chassis.getLeftPID(chassis.getLeftVelocity()) + chassis.getLeftF());
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}