package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase  {

  private MATalonSRX talon1;
  private MATalonSRX talon2;

  private ColorSensorV3 ir1;
  private ColorSensorV3 ir2;

  private static Conveyor conveyor;

  private MAShuffleboard conveyorShuffleboard;


  public Conveyor() {
    talon1 = new MATalonSRX(5,false, 0,false,false, false, null);
    talon2 = new MATalonSRX(11,false, 0,false,false, false, null);

    ir1= new ColorSensorV3(Port.kOnboard);
    ir2=new ColorSensorV3(Port.kMXP);

    shooterShuffleboard = new MAShuffleboard("conveyor");
  }

  public int getIR1Values(){
    return ir1.getIR();
  }

  public int getIR2Values(){
    return ir2.getIR();
  }


  public boolean getIR1(){
    return getIR1Values() > 18;
  }

  public boolean getIR2(){
    return getIR2Values() > 22;
  }

  public void setTalon1Velocity(double velocity){
    talon1.set(ControlMode.PercentOutput, velocity);
  }

  public void setTalon2Velocity(double velocity){
    talon2.set(ControlMode.PercentOutput, velocity);
  }

  public static Conveyor getinstance(){
    if (conveyor == null){
      conveyor = new Conveyor();
    }
    return conveyor;
  } 

  @Override
  public void periodic() {

    conveyorShuffleboard.putNumber("ir1Values",getIR1Values());
    conveyorShuffleboard.putBoolean("ir1",getIR1());

    conveyorShuffleboard.putNumber("ir2Values", getIR2Values());
    conveyorShuffleboard.putBoolean("ir2", getIR2());
  }
}