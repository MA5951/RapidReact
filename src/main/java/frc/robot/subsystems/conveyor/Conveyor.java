package frc.robot.subsystems.conveyor;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LEDManager;
import frc.robot.PortMap;
import com.ma5951.utils.Shuffleboard;
import com.ma5951.utils.motor.MA_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

public class Conveyor extends SubsystemBase {

    private MA_TalonSRX lowerTalon;
    private TalonFX upperFalcon;

    private DigitalInput lowerIR;
    private DigitalInput upperIR;
    private ColorSensorV3 colorSensor;
    private boolean isOurTeamColorBlue;

    private I2C.Port i2cPort = I2C.Port.kOnboard;
    public boolean isBallInUpper = false;
    private double lastLEDTime = 0;
    public boolean isInControlLED = true;
    public boolean isInAutonomous = true;
    private static Conveyor conveyor;
    private   ColorMatch m_colorMatcher = new ColorMatch();

    private  Color kBlueTarget = new Color(0.143, 0.427, 0.429);
    private  Color kRedTarget = new Color(0.561, 0.232, 0.114);

    private Shuffleboard conveyorShuffleboard;

    private int amountOfBalls;

    public Conveyor() {
        lowerTalon = new MA_TalonSRX(PortMap.conveyorLowerMotor, false, false); // ID6
        upperFalcon = new TalonFX(PortMap.conveyorUpperMotor); // ID7

        lowerIR = new DigitalInput(PortMap.conveyorLowerIR);
        upperIR = new DigitalInput(PortMap.conveyorUpperIR);

        colorSensor = new ColorSensorV3(i2cPort);

        lowerTalon.configRampRate(ConveyorConstants.CONVEYOR_LOWER_RAMP_RATE);

        conveyorShuffleboard = new Shuffleboard("conveyor");

        amountOfBalls = 0;
    }

    public void setOurTeamColorBlue(boolean isOurTeamColorBlue) {
        this.isOurTeamColorBlue = isOurTeamColorBlue;
    }

    public boolean getOurTeamColorBlue() {
        return isOurTeamColorBlue;
    }

    public boolean isBallInLower() {
        return isBlue() || isRed();
    }

    public boolean isBallInUpper() {
        return (!upperIR.get() || !lowerIR.get());
    }

    public void setLowerPower(double velocity) {
        lowerTalon.setPower(velocity);
    }

    public void setUpperPower(double velocity) {
        upperFalcon.set(TalonFXControlMode.PercentOutput, velocity);
    }

    public void setAmountOfBalls(int numBalls) {
        if ((numBalls) <= ConveyorConstants.CONVEYOR_MAX_BALLS && numBalls >= 0) {
            amountOfBalls = numBalls;
        }
    }

    public int getAmountOfBalls() {
        return amountOfBalls;
    }

    public boolean isRed(){
        /*
        Color detectedColor = colorSensor.getColor();
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
        if(match.color == kRedTarget){
            return true;
        }
        else{
            return false;
        }*/
        if(colorSensor.getRed() > 2000 && colorSensor.getBlue() < 1000){
            return true;
        }
        return false;
    }

    public boolean isBlue(){
        /*
        Color detectedColor = colorSensor.getColor();
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
        if(match.color == kBlueTarget){
            return true;
        }
        else{
            return false;
        }
        */
        if(colorSensor.getBlue() > 2000 && colorSensor.getRed() < 2000){  
            return true;
        }
        return false;
    }

    public static Conveyor getInstance() {
        if (conveyor == null) {
            conveyor = new Conveyor();
        }
        return conveyor;
    }

    public double getUpperStator() {
        return upperFalcon.getStatorCurrent();
    }

    @Override
    public void periodic() {
        conveyorShuffleboard.addBoolean("isBallInLower", isBallInLower());
        conveyorShuffleboard.addBoolean("isBallInUpper", isBallInUpper());
        conveyorShuffleboard.addNum("amount of balls", amountOfBalls);
        conveyorShuffleboard.addNum("blue", colorSensor.getBlue());
        conveyorShuffleboard.addNum("red", colorSensor.getRed());
        conveyorShuffleboard.addNum("green", colorSensor.getGreen());
        conveyorShuffleboard.addBoolean("isblue", isBlue());
        conveyorShuffleboard.addBoolean("isRed", isRed());
        conveyorShuffleboard.addNum("Current", upperFalcon.getStatorCurrent());

        if (isInControlLED) {
            if (DriverStation.getMatchType() == MatchType.None
                    || (DriverStation.getMatchTime() > 5 || isInAutonomous)) {
                double current = Timer.getFPGATimestamp();
                if (current - lastLEDTime > 0.95) {
                    if (isBallInUpper() && isBallInLower()) {
                        LEDManager.getInstance().setGreen();
                    } else if (isBallInUpper() || isBallInLower()) {
                        LEDManager.getInstance().setOrange();
                        lastLEDTime = current;
                    } else {
                        LEDManager.getInstance().setRed();
                        lastLEDTime = current;
                    }
                }
            } else {
                LEDManager.getInstance().setRainbow();
            }
        }
    }
}