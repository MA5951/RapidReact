package frc.robot.commands.chassis;

import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.utils.autonomous.OdometryHandler;
import frc.robot.utils.autonomous.Path;
import frc.robot.utils.autonomous.PathFollower;
import frc.robot.utils.autonomous.PathGenerator;
import frc.robot.utils.autonomous.Waypoint;

public class AutonomousCommand extends CommandBase {

  PathGenerator pathGenerator;
  PathFollower pathFollower;
  OdometryHandler odometry;
  double leftSetPointVelocity;
  double rightSetPointVelocity;

  double leftVelocity;
  double rightVelocity;

  double maxVelocity = 2.2; // 2.3
  double maxAcceleration = 0.33;

  private Chassis chassis;

  public AutonomousCommand(Path path) {
    chassis = Chassis.getinstance();
    odometry = chassis.getOdomoteryHandler();

    pathGenerator = new PathGenerator(path.spacing, path.k, path.maxVelocity, path.maxAcceleration);
    List<Waypoint> waypoints = (List<Waypoint>) pathGenerator.calculate(path.points);
    pathFollower = new PathFollower(waypoints, odometry, path.lookaheadDistance, 
                                    path.maxRate, 0.75);    
    addRequirements(chassis);
  }


  @Override
  public void initialize()  {
    chassis.resetSensors();
    chassis.setIdleMode(NeutralMode.Brake);
  }

  @Override
  public void execute() {
    double[] speeds = pathFollower.getSpeeds();
    
    chassis.chassisShuffleboard.addNum("linear left speed", speeds[0]);
    chassis.chassisShuffleboard.addNum("linear right speed", speeds[1]);

    leftSetPointVelocity  = speeds[0];
    rightSetPointVelocity = speeds[1];

    chassis.chassisShuffleboard.addNum("left speed", leftSetPointVelocity);
    chassis.chassisShuffleboard.addNum("right speed", rightSetPointVelocity);

    // chassis.setLeftVelocitySetpoint(leftSetPointVelocity);
    // chassis.setRightVelocitySetpoint(-rightSetPointVelocity);

    chassis.setLeftVelocitySetpoint(leftSetPointVelocity);
    chassis.setRightVelocitySetpoint(rightSetPointVelocity);

    leftVelocity  = MathUtil.clamp(chassis.leftVelocityMApathPIDOutput()+chassis.getLeftF(), -1, 1);
    rightVelocity = MathUtil.clamp(chassis.rightVelocityMApathPIDOutput()+chassis.getRightF(), -1, 1);

    // System.out.println("left: " + []\
    // leftVelocity + "right: " + rightVelocity);

    chassis.setLeftPercent(leftVelocity + 0.05);
    chassis.setRightPercent(rightVelocity + 0.05);
  }

  @Override
  public void end(boolean interrupted) {
    chassis.setLeftVoltage(0);
    chassis.setRightVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return pathFollower.isDone();
  }
}