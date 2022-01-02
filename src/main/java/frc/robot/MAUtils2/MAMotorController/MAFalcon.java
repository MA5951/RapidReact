package frc.robot.MAUtils2.MAMotorController;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class MAFalcon implements MAMotorControlInterface {

    private TalonFX falcon;

    public MAFalcon(int id, boolean inverted, double rampRate, NeutralMode mode, boolean hasForwardLimitSwitch,
        boolean hasReverseLimitSwitch, FeedbackDevice feedbackDevice) {
        falcon = new TalonFX(id);
        setInverted(inverted);
        configRampRate(rampRate);
        falcon.setNeutralMode(mode);
        

        if (hasForwardLimitSwitch)
            configForwardLimitSwitchSource();

        if (hasReverseLimitSwitch)
            configReverseLimitSwitchSource();

        falcon.configSelectedFeedbackSensor(feedbackDevice);

    }

    private void configForwardLimitSwitchSource() {
        falcon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    }

    private void configReverseLimitSwitchSource() {
        falcon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    }

    @Override
    public void setvoltage(double voltage) {
        falcon.set(TalonFXControlMode.PercentOutput, voltage / 12);

    }

    @Override
    public void set(double Power) {
        falcon.set(TalonFXControlMode.PercentOutput, Power);

    }

    @Override
    public double getOutPut() {
        return falcon.getMotorOutputPercent();
    }

    @Override
    public void configRampRate(double rampRate) {
        falcon.configOpenloopRamp(rampRate);
        falcon.configClosedloopRamp(rampRate);

    }

    public void setInverted(Boolean setInverted) {
        falcon.setInverted(setInverted);

    }

    @Override
    public void changeMode(boolean onOff) {
        falcon.setNeutralMode(onOff ? NeutralMode.Brake : NeutralMode.Coast);

    }

    public void resetEncoder() {
        falcon.setSelectedSensorPosition(0);
    }

    public int getForwardLimitSwitch() {
        return falcon.getSensorCollection().isFwdLimitSwitchClosed();
    }

    public int getReverseLimitSwitch() {
        return falcon.getSensorCollection().isRevLimitSwitchClosed();
    }

    public double getPosition() {
        return falcon.getSelectedSensorPosition();
    }

    public double getVelocity() {
        return falcon.getSelectedSensorVelocity();
    }

    public double getStatorCurrent() {
        return falcon.getStatorCurrent();
    }

    public void phaseSensor(boolean PhaseSensor) {
        falcon.setSensorPhase(PhaseSensor);
    }

    public void resatOnLimitF(boolean limit) {
        falcon.configClearPositionOnLimitF(limit, 0);
    }

    public void resatOnLimitR(boolean limit) {
        falcon.configClearPositionOnLimitR(limit, 0);
    }

    public void DisableLimit(boolean onOff) {
        falcon.configLimitSwitchDisableNeutralOnLOS(onOff, 0);
    }

    public void overrideLimitSwitches(boolean overrid) {
        falcon.overrideLimitSwitchesEnable(overrid);
    }

    @Override
    public int getID() {
        return falcon.getDeviceID();
    }

    public void follow(IMotorController Motor) {
        falcon.follow(Motor);
    }

    public void setFeedBack(FeedbackDevice FD) {
        falcon.configSelectedFeedbackSensor(FD);
    }
}
