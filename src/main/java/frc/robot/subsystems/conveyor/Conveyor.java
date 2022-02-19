package frc.robot.subsystems.conveyor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import com.ma5951.utils.Shuffleboard;
import com.ma5951.utils.RobotConstants;
import com.ma5951.utils.motor.MA_VictorSPX;

public class Conveyor extends SubsystemBase {

    private MA_VictorSPX lowerVictor;
    private MA_VictorSPX upperVictor;

    private DigitalInput lowerIR;
    private DigitalInput upperIR;

    private static Conveyor conveyor;

    private Shuffleboard conveyorShuffleboard;

    private int amountOfBalls;

    public Conveyor() {
        lowerVictor = new MA_VictorSPX(PortMap.conveyorLowerMotor, false, RobotConstants.KMOTOR_COAST); // ID6
        upperVictor = new MA_VictorSPX(PortMap.conveyorUpperMotor, false, RobotConstants.KMOTOR_COAST); // ID7

        lowerIR = new DigitalInput(PortMap.conveyorLowerIR);
        upperIR = new DigitalInput(PortMap.conveyorUpperIR);

        lowerVictor.configRampRate(ConveyorConstants.CONVEYOR_LOWER_RAMP_RATE);

        conveyorShuffleboard = new Shuffleboard("conveyor");
    }

    public boolean isBallInLower() {
        return lowerIR.get();
    }

    public boolean isBallInUpper() {
        return upperIR.get();
    }

    public void setLowerPower(double velocity) {
        lowerVictor.setPower(velocity);
    }

    public void setUpperPower(double velocity) {
        upperVictor.setPower(velocity);
    }

    public void setAmountOfBalls(int numBalls) {
        if ((getAmountOfBalls() + numBalls) <= ConveyorConstants.CONVEYOR_MAX_BALLS) {
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