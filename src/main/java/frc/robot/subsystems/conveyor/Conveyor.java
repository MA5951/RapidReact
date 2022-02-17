package frc.robot.subsystems.conveyor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.utils.MAShuffleboard;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.motor.MAVictorSPX;

public class Conveyor extends SubsystemBase {

  private MAVictorSPX lowerVictor;
  private MAVictorSPX upperVictor;

  private DigitalInput lowerIR;
  private DigitalInput upperIR;

  private static Conveyor conveyor;

  private MAShuffleboard conveyorShuffleboard;

  private int amountOfBalls;

  public Conveyor() {
    lowerVictor = new MAVictorSPX(PortMap.conveyorLowerMotor, false, RobotConstants.KMOTOR_COAST); //ID6
    upperVictor = new MAVictorSPX(PortMap.conveyorUpperMotor, false, RobotConstants.KMOTOR_COAST); //ID7

    lowerIR = new DigitalInput(1);
    upperIR = new DigitalInput(2);

    lowerVictor.configRampRate(0.1);

    conveyorShuffleboard = new MAShuffleboard("conveyor");
  }

  public boolean isBallInLower() {
    return lowerIR.get();
  }

  public boolean isBallInUpper() {
    return upperIR.get();
  }

  public void setLowerPower(double velocity) {
    lowerVictor.set(velocity);
  }

  public void setUpperPower(double velocity) {
    upperVictor.set(velocity);
  }

  public void setAmountOfBalls(int numBalls) {
    if ((getAmountOfBalls() + numBalls) <= 2) {
      amountOfBalls = numBalls;
    }
  }

  public int getAmountOfBalls() {
    return amountOfBalls;
  }

  public static Conveyor getInstance() {
    if (conveyor == null) {
      conveyor = new Conveyor();
    }
    return conveyor;
  }

  @Override
  public void periodic() {

    conveyorShuffleboard.addBoolean("ir1", isBallInLower());

    conveyorShuffleboard.addBoolean("ir2", isBallInUpper());
  }
}