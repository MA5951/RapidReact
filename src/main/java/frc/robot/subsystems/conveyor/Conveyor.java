package frc.robot.subsystems.conveyor;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MAShuffleboard;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.motor.MAVictorSPX;

public class Conveyor extends SubsystemBase {

  private MAVictorSPX lowerVictor;
  private MAVictorSPX upperVictor;

  private DigitalInput ir1;
  private DigitalInput ir2;

  private static Conveyor conveyor;

  private MAShuffleboard conveyorShuffleboard;

  public int amoutOfBalls;

  public Conveyor() {
    lowerVictor = new MAVictorSPX(RobotConstants.ID6, false, RobotConstants.KMOTOR_COAST); //ID6
    upperVictor = new MAVictorSPX(RobotConstants.ID7, false, RobotConstants.KMOTOR_COAST); //ID7

    ir1 = new DigitalInput(1);
    ir2 = new DigitalInput(2);

    lowerVictor.configRampRate(0.1);

    conveyorShuffleboard = new MAShuffleboard("conveyor");
  }

  public boolean getIR1() {
    return ir1.get();
  }

  public boolean getIR2() {
    return ir2.get();
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

    conveyorShuffleboard.addBoolean("ir1", getIR1());

    conveyorShuffleboard.addBoolean("ir2", getIR2());
  }
}