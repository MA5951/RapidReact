package frc.robot.commands.chassis;

import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.ChassisConstants;

import com.ma5951.utils.autonomous.*;

public class AutonomousCommand extends CommandBase {
  /**
   *  Autonomous Movement
   */

  PathGenerator pathGenerator;
  PathFollower pathFollower;
  OdometryHandler odometry;
  double leftSetPointVelocity;
  double rightSetPointVelocity;

  double leftVelocity;
  double rightVelocity;

  double maxVelocity = ChassisConstants.MAX_VELOCITY; // 2.3
  double maxAcceleration = ChassisConstants.MAX_ACCELERATION;

  private boolean reverse;

  private Chassis chassis;

  private List<Waypoint> waypoints;

  private Path path;

  public AutonomousCommand(Path CHECKING_PATH, boolean reverse) {
    path = CHECKING_PATH;
    chassis = Chassis.getinstance();
    odometry = chassis.getOdomoteryHandler();

    pathGenerator = new PathGenerator(path.spacing, path.k, path.maxVelocity, path.maxAcceleration);
    waypoints = (List<Waypoint>) pathGenerator.calculate(path.points);
    
    this.reverse = reverse;

    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    pathFollower = new PathFollower(waypoints, odometry, path.lookaheadDistance,
        path.maxRate, ChassisConstants.TRACK_WIDTH);
    chassis.chassisShuffleboard.addBoolean("Autonomus Test", true);
    chassis.resetSensors();
    chassis.setIdleMode(NeutralMode.Brake);
    chassis.setInverted(reverse);
  }

  @Override
  public void execute() {
    double[] speeds = pathFollower.getSpeeds();

    chassis.chassisShuffleboard.addNum("linear left speed", speeds[0]);
    chassis.chassisShuffleboard.addNum("linear right speed", speeds[1]);
    chassis.chassisShuffleboard.addNum("curvature", pathFollower.getLookaheadPoint().curvature);

    leftSetPointVelocity = speeds[0];
    rightSetPointVelocity = speeds[1];

    chassis.chassisShuffleboard.addNum("left speed", leftSetPointVelocity);
    chassis.chassisShuffleboard.addNum("right speed", rightSetPointVelocity);

    chassis.chassisShuffleboard.addString("lookhaed point", "(" +
     pathFollower.getLookaheadPoint().x + "," +
      pathFollower.getLookaheadPoint().y + ")");

    chassis.setLeftVelocitySetpoint(leftSetPointVelocity);
    chassis.setRightVelocitySetpoint(rightSetPointVelocity);

    leftVelocity = MathUtil.clamp(
        chassis.leftVelocityMApathPIDOutput() + chassis.getLeftF() + ChassisConstants.KV_MAPATH_LEFT_VELOCITY, -1, 1);
    rightVelocity = MathUtil.clamp(
        chassis.rightVelocityMApathPIDOutput() + chassis.getRightF() + ChassisConstants.KV_MAPATH_RIGHT_VELOCITY, -1,
        1);

    chassis.setLeftPercent(leftVelocity);
    chassis.setRightPercent(rightVelocity);
    System.out.println("exec");
  }

  @Override
  public void end(boolean interrupted) {
    chassis.chassisShuffleboard.addBoolean("Autonomus Test", false);
    System.out.println("end");
    chassis.setLeftVoltage(0);
    chassis.setRightVoltage(0);
    chassis.setInverted(false);
  }

  @Override
  public boolean isFinished() {
    return pathFollower.isDone();
  }
}