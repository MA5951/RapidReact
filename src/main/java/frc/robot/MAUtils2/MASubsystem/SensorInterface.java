package frc.robot.MAUtils2.MASubsystem;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SensorInterface{
    public boolean canMove();

    public void reset();

    public Subsystem getInstance();
}
