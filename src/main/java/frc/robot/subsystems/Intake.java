// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MAUtils2.RobotConstants;
import frc.robot.MAUtils2.MAMotorController.MATalonSRX;

public class Intake extends SubsystemBase {
  /** Creates a new Instake. */
  private MATalonSRX intakeMotor;
  private static Intake intake;

  public Intake() {
    intakeMotor = new MATalonSRX(RobotConstants.ID10, false, 0, true, false, false, FeedbackDevice.None); //ID5
  }

  public void setPower(double power) {
    intakeMotor.set(power);
  }


  public static Intake getinstance(){
    if (intake == null){
      intake = new Intake();
    }
    return intake;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
