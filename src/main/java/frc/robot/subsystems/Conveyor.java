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

  private MAVictorSPX talon1;
  private MAVictorSPX talon2;

  private ColorSensorV3 ir1;
  private ColorSensorV3 ir2;

  private static Conveyor conveyor;

  private MAShuffleboard conveyorShuffleboard;

  public Conveyor() {
    talon1 = new MAVictorSPX(RobotConstants.ID6, false, RobotConstants.KMOTOR_BRAKE);
    talon2 = new MAVictorSPX(RobotConstants.ID7, false, RobotConstants.KMOTOR_BRAKE);

    ir1 = new ColorSensorV3(Port.kOnboard);
    ir2 = new ColorSensorV3(Port.kMXP);

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
    return getIR2Values() > 22;
  }

  public void setTalon1Velocity(double velocity) {
    talon1.set(velocity);
  }

  public void setTalon2Velocity(double velocity) {
    talon2.set(velocity);
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