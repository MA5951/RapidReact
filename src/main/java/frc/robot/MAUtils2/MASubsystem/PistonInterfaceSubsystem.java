package frc.robot.MAUtils2.MASubsystem;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface PistonInterfaceSubsystem extends Subsystem {
    public void open();

    public void close();

    public boolean isOpen();

}