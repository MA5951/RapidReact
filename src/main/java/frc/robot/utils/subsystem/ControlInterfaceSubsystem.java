package frc.robot.utils.subsystem;

public interface ControlInterfaceSubsystem extends MotorInterfaceSubsystem{
    public void setSetpoint(double setPoint);

    public boolean atSetpoint();

    public double calculate();
}