package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.MAUtils2.MAShuffleboard;
import frc.robot.MAUtils2.RobotConstants;
import frc.robot.MAUtils2.MAMotorController.MATalonSRX;
import frc.robot.MAUtils2.MAMotorController.MAVictorSPX;

public class Conveyor extends SubsystemBase {

  private MAVictorSPX lowerVictor;
  private MAVictorSPX upperVictor;

  private ColorSensorV3 ir1;
  private ColorSensorV3 ir2;

  private static Conveyor conveyor;

  private MAShuffleboard conveyorShuffleboard;

  public int amoutOfBalls;

  public Conveyor() {
    lowerVictor = new MAVictorSPX(RobotConstants.ID6, false, RobotConstants.KMOTOR_COAST); //ID6
    upperVictor = new MAVictorSPX(RobotConstants.ID7, false, RobotConstants.KMOTOR_COAST); //ID7

    ir1 = new ColorSensorV3(Port.kOnboard);
    ir2 = new ColorSensorV3(Port.kMXP);

    lowerVictor.configRampRate(0.1);

    conveyorShuffleboard = new MAShuffleboard("conveyor");
  }

  public int getIR1Values() {
    return ir1.getIR();
  }

  public int getIR2Values() {
    return ir2.getIR();
  }

  public boolean getIR1() {
    return getIR1Values() > 18;
  }

  public boolean getIR2() {
    return getIR2Values() > 23;
  }

  public void setLowerPower(double velocity) {
    lowerVictor.set(velocity);
  }

  public void setUpperPower(double velocity) {
    upperVictor.set(velocity);
  }

  public static Conveyor getinstance() {
    if (conveyor == null) {
      conveyor = new Conveyor();
    }
    return conveyor;
  }

  @Override
  public void periodic() {

    conveyorShuffleboard.addNum("ir1Values", getIR1Values());
    conveyorShuffleboard.addBoolean("ir1", getIR1());

    conveyorShuffleboard.addNum("ir2Values", getIR2Values());
    conveyorShuffleboard.addBoolean("ir2", getIR2());
  }
}