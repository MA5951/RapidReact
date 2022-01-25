package frc.robot.MAUtils2.MASubsystem;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface MotorInterfaceSubsystem extends Subsystem {
    public void setVoltege (double voltege);
    
    default void setPower(double power){
        setVoltege(power * 12);
    }

    public double getVoltege();

    default double getPower(){
        return getVoltege() * 12;
    }
}

 