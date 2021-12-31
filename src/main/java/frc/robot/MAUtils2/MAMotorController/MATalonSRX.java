// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MAUtils2.MAMotorController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

class MATalonSRX implements MAMotorControlInterface{
    private WPI_TalonSRX talonSRX;

    public MATalonSRX(int id, boolean inverted, double rampRate, boolean mod, boolean hasForwardLimitSwitch,
            boolean hasReverseLimitSwitch, FeedbackDevice feedbackDevice) {
        talonSRX = new WPI_TalonSRX(id);
        setInverted(inverted);
        configRampRate(rampRate);
        changeMode(mod);

        if (hasForwardLimitSwitch)
            configForwardLimitSwitchSource();

        if (hasReverseLimitSwitch)
            configReverseLimitSwitchSource();

        talonSRX.configSelectedFeedbackSensor(feedbackDevice);

    }

    private void configForwardLimitSwitchSource() {
        talonSRX.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    }

    private void configReverseLimitSwitchSource() {
        talonSRX.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    }

    @Override
    public void setvoltage(double voltage) {
        talonSRX.setVoltage(voltage);

    }

    @Override
    public void set(double Power) {
        talonSRX.set(ControlMode.PercentOutput, Power);

    }

    @Override
    public double getOutPut() {
        return talonSRX.get();
    }

    @Override
    public void configRampRate(double rampRate) {
        talonSRX.configOpenloopRamp(rampRate);
        talonSRX.configClosedloopRamp(rampRate);

    }

    @Override
    public void setInverted(Boolean setInverted) {
        talonSRX.setInverted(setInverted);

    }

    @Override
    public void changeMode(boolean onOff) {
        talonSRX.setNeutralMode(onOff ? NeutralMode.Brake : NeutralMode.Coast);

    }


    public void resetEncoder() {
        talonSRX.setSelectedSensorPosition(0);
    }

    public boolean getForwardLimitSwitch() {
        return talonSRX.getSensorCollection().isFwdLimitSwitchClosed();
    }

    public boolean getReverseLimitSwitch() {
        return talonSRX.getSensorCollection().isRevLimitSwitchClosed();
    }

    public double getPosition() {
        return talonSRX.getSelectedSensorPosition();
    }

    public double getVelocity() {
        return talonSRX.getSelectedSensorVelocity();
    }

    public double getStatorCurrent() {
        return talonSRX.getStatorCurrent();
    }

    public void phaseSensor(boolean PhaseSensor) {
        talonSRX.setSensorPhase(PhaseSensor);
    }

    public void resetOnLimitF(boolean limit) {
        talonSRX.configClearPositionOnLimitF(limit, 0);
    }

    public void resetOnLimitR(boolean limit) {
        talonSRX.configClearPositionOnLimitR(limit, 0);
    }

    public void DisableLimit(boolean onOff) {
        talonSRX.configLimitSwitchDisableNeutralOnLOS(onOff, 0);
    }

    public void overrideLimitSwitches(boolean overrid) {
        talonSRX.overrideLimitSwitchesEnable(overrid);
    }

    @Override
    public int getID() {
        return talonSRX.getDeviceID();
    }

    public IMotorController getIMotorController() {
        return talonSRX;
    }

    public void follow(IMotorController Motor) {
        talonSRX.follow(Motor);
    }

    public void setFeedBack(FeedbackDevice FD) {
        talonSRX.configSelectedFeedbackSensor(FD);
    }

    public void setCurrentLimit(int limit) {
        talonSRX.configContinuousCurrentLimit(limit);
        talonSRX.configPeakCurrentDuration(0);

    }

}