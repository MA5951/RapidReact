// package frc.robot.commands.Chassis;

// import java.util.ArrayList;

// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.fasterxml.jackson.databind.util.NameTransformer.Chained;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Chassis;
// import frc.robot.commands.Chassis.ChassisCostan;
// import frc.robot.utils.Autonomous.OdometryHandler;
// import frc.robot.utils.Autonomous.PathFollower;
// import frc.robot.utils.Autonomous.Waypoint;
// import frc.robot.utils.Autonomous.PathGenerator;
// import frc.robot.utils.Calculation.MACalculations;

// public class AutonomousCommand extends CommandBase {

//   PathGenerator pathGenerator;
//   PathFollower pathFollower;
//   OdometryHandler odometry;
//   double leftSetPointVelocity;
//   double rightSetPointVelocity;

//   double leftVelocity;
//   double rightVelocity;

//   double maxVelocity = 2.3;
//   double maxAcceleration = 1.5;

//   private Chassis chassis;

//   private static ArrayList<Waypoint> autoPath;
//   private ArrayList<Waypoint> path;

//   public AutonomousCommand() {
//     pathGenerator = new PathGenerator(0.1, 1, maxVelocity, maxAcceleration);
//     System.out.println("generator construction");

//     chassis = Chassis.getinstance();
//     System.out.println("chassis");
//     odometry = chassis.getOdomoteryHandler();
//     System.out.println("odometry");
    

//     autoPath = new ArrayList<Waypoint>();
//     autoPath.add(new Waypoint(0, 0));
//     autoPath.add(new Waypoint(0, -1));
//     autoPath.add(new Waypoint(-0.2, -3));


//     path = (ArrayList<Waypoint>) pathGenerator.calculate(autoPath);
//     System.out.println("generate");

//     pathFollower = new PathFollower(path, odometry, 0.15, 0.8, 0.25, 0.75);
//     System.out.println("follower");

//     chassis.resetSensors();

//     addRequirements(chassis);
//   }


//   @Override
//   public void initialize()  {
//     System.out.println("init");
//     chassis.resetSensors();
//     chassis.setIdleMode(NeutralMode.Brake);
//   }

//   @Override
//   public void execute() {
//     for (int i = 0; i < path.size(); i++) {
//       chassis.chassisShuffleboard.addString("Point " + i, path.get(i).toString());
//     }

//     chassis.chassisShuffleboard.addString("closest point", pathFollower.getClosestPoint().toString());
//     chassis.chassisShuffleboard.addString("lookahead point", pathFollower.getLookaheadPoint().toString());


//     double[] speeds = pathFollower.getSpeeds();
    
//     chassis.chassisShuffleboard.addNum("linear left speed", speeds[0]);
//     chassis.chassisShuffleboard.addNum("linear right speed", speeds[1]);

//     leftSetPointVelocity  = speeds[0];
//     rightSetPointVelocity = speeds[1];

//     chassis.chassisShuffleboard.addNum("left speed", leftSetPointVelocity);
//     chassis.chassisShuffleboard.addNum("right speed", rightSetPointVelocity);

//     // chassis.setLeftVelocitySetpoint(leftSetPointVelocity);
//     // chassis.setRightVelocitySetpoint(-rightSetPointVelocity);

//     chassis.setLeftVelocitySetpoint(leftSetPointVelocity);
//     chassis.setRightVelocitySetpoint(rightSetPointVelocity);

//     leftVelocity  = MathUtil.clamp(chassis.leftVelocityMApathPIDOutput()+chassis.getLeftF(), -1, 1);
//     rightVelocity = MathUtil.clamp(chassis.rightVelocityMApathPIDOutput()+chassis.getRightF(), -1, 1);

//     // System.out.println("left: " + leftVelocity + "right: " + rightVelocity);

//     // chassis.setLeftPercent(leftVelocity + 0.05);
//     // chassis.setRightPercent(rightVelocity + 0.05);
//   }

//   @Override
//   public void end(boolean interrupted) {
//     chassis.setLeftVoltage(0);
//     chassis.setRightVoltage(0);
//     System.out.println("end");
//     System.out.println(chassis.getLeftEncoder());
//     System.out.println(chassis.getRightEncoder());
//     System.out.println(chassis.getAngle());
//   }

//   @Override
//   public boolean isFinished() {
//     //return false;
//     return pathFollower.isDone();
//   }
// }
