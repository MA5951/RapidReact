// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autonomous.AutonomousPaths.BluePathAutonomous;
import frc.robot.autonomous.AutonomousPaths.GreenPathAutonomous;
import frc.robot.autonomous.AutonomousPaths.RedPathAutonomous;
import frc.robot.commands.Automations.*;
import frc.robot.commands.shooter.EjectBall;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.climb.ClimbExtension;
import frc.robot.subsystems.climb.ClimbPassive;
import frc.robot.subsystems.climb.ClimbRotation;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

import com.ma5951.utils.JoystickContainer;
import com.ma5951.utils.RobotConstants;
import com.ma5951.utils.commands.ControlCommand;
import com.ma5951.utils.commands.MotorCommand;
import com.ma5951.utils.commands.PistonCommand;
import com.ma5951.utils.commands.TogglePistonCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public static boolean robotControlMode;
    public static Joystick leftJoystick = new Joystick(RobotConstants.KLEFT_JOYSTICK_PORT);
    public static Joystick righJoystick = new Joystick(RobotConstants.KRIGHT_JOYSTICK_PORT);
    public static Trigger leftTrigger = new Trigger(
            () -> JoystickContainer.operatingJoystick.getRawAxis(RobotConstants.L_TRIGER) > 0.5);
    public static Trigger rightTrigger = new Trigger(
            () -> JoystickContainer.operatingJoystick.getRawAxis(RobotConstants.R_TRIGER) > 0.5);

    JoystickButton controlModeToogle;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        // ---------------------------- Intake & Conveyor ----------------------------
        JoystickContainer.AButton.whileActiveContinuous(new IntakeAutomation(0.8));

        JoystickContainer.XButton.whileActiveContinuous(
                () -> Conveyor.getInstance().setLowerPower(0.7));
        JoystickContainer.XButton.whileActiveContinuous(
                () -> Intake.getInstance().setPower(-0.8));
        JoystickContainer.XButton.whenReleased(() -> Intake.getInstance().setPower(0));
        JoystickContainer.XButton.whenReleased(() -> Conveyor.getInstance().setLowerPower(0));
        JoystickContainer.XButton.whenPressed(new InstantCommand(
                () -> Conveyor.getInstance()
                        .setAmountOfBalls(Conveyor.getInstance().getAmountOfBalls() - 1)));

        new Trigger(() -> JoystickContainer.leftJoystick.getRawButton(2))
                .whileActiveContinuous(() -> Intake.getInstance().setPower(-0.7));

        new Trigger(() -> JoystickContainer.leftJoystick.getRawButton(2))
                .whenInactive(() -> Intake.getInstance().setPower(0));
        new Trigger(() -> JoystickContainer.leftJoystick.getRawButton(2))
                .whenInactive(() -> Conveyor.getInstance().setUpperPower(0));
        new Trigger(() -> JoystickContainer.leftJoystick.getRawButton(2))
                .whenInactive(() -> Conveyor.getInstance().setLowerPower(0));
        new Trigger(() -> JoystickContainer.leftJoystick.getRawButton(2))
                .whenInactive(() -> Conveyor.getInstance().isBallInUpper = false);
        new Trigger(() -> JoystickContainer.leftJoystick.getRawButton(2))
                .whileActiveContinuous(() -> Conveyor.getInstance().setLowerPower(0.7));
        new Trigger(() -> JoystickContainer.leftJoystick.getRawButton(2))
                .whileActiveContinuous(() -> Conveyor.getInstance().setUpperPower(0.8));
        new Trigger(() -> JoystickContainer.leftJoystick.getRawButton(2))
                .whileActiveContinuous(() -> Conveyor.getInstance().setAmountOfBalls(0));

        // ---------------------------- Shooter ------------ ----------------
        new Trigger(() -> JoystickContainer.rightJoystick.getRawButton(2))
                .whileActiveContinuous(new ShooterAutomation());

        leftTrigger.whileActiveContinuous(new EjectBall(ShooterConstants.SHOOTER_VELOCITY_FENDER)
                .alongWith(new UpperConveyorCommand()));

        new Trigger(() -> JoystickContainer.rightJoystick.getRawButton(3))
                .whileActiveContinuous(new MotorCommand(Intake.getInstance(), -0.7));

        JoystickContainer.YButton.whenPressed(new PistonCommand(Intake.getInstance(), false));

        JoystickContainer.BButton.whileActiveContinuous(new EjectBall(-1000));
        JoystickContainer.BButton.whileActiveContinuous(() -> Conveyor.getInstance().setUpperPower(-0.9));
        JoystickContainer.BButton.whenReleased(() -> Conveyor.getInstance().setUpperPower(0));
        JoystickContainer.BButton.whenPressed(new InstantCommand(
                () -> Conveyor.getInstance()
                        .setAmountOfBalls(Conveyor.getInstance().getAmountOfBalls() - 1)));
        JoystickContainer.BButton.whenPressed(() -> Conveyor.getInstance().isBallInUpper = false);

        // ---------------------------- Climb ----------------------------
        new Trigger(() -> JoystickContainer.leftJoystick.getRawButton(2))
                .whileActiveContinuous(new ControlCommand(ClimbRotation.getInstance(), 0, false, true));

        JoystickContainer.POVUp
                .whenActive(new ControlCommand(ClimbExtension.getInstance(),
                        ClimbConstants.MAX_POSITION, true, true)
                                .alongWith(new InstantCommand(
                                        LEDManager.getInstance()::setRainbow)));

        JoystickContainer.POVDown.whenActive(new climbAutomationToSecond());

        JoystickContainer.POVLeft.whenActive(new ClimbCloseAutomation());

        JoystickContainer.POVRight.whenActive(new ClimbAutomationToThird());

        JoystickContainer.LB.whileActiveContinuous(new MotorCommand(ClimbPassive.getInstance(), -0.4));

        JoystickContainer.RB.whileActiveContinuous(new MotorCommand(ClimbPassive.getInstance(), 0.4));

        JoystickContainer.startButton
                .whenPressed(
                        () -> Conveyor.getInstance().setAmountOfBalls(
                                Conveyor.getInstance().getAmountOfBalls() + 1));
        JoystickContainer.backButton
                .whenPressed(
                        () -> Conveyor.getInstance().setAmountOfBalls(
                                Conveyor.getInstance().getAmountOfBalls() - 1));

        // ---------------------------- Emergency buttons ----------------------------
        new Trigger(() -> JoystickContainer.leftJoystick.getRawButton(1))
                .whenActive(new TogglePistonCommand(Intake.getInstance()));

        new Trigger(() -> JoystickContainer.rightJoystick.getRawButton(5))
                .whileActiveContinuous(new IntakeAutomation(0.8));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new GreenPathAutonomous();
    }
}
