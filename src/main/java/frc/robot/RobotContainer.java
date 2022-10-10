// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autonomous.AutonomousPaths.BluePathAutonomous;
import frc.robot.autonomous.AutonomousPaths.GreenPathAutonomous;
import frc.robot.autonomous.AutonomousPaths.RedPathAutonomous;
import frc.robot.commands.Automations.*;
import frc.robot.commands.conveyor.ConveyorCommand;
import frc.robot.commands.conveyor.ConveyorCommandColor;
import frc.robot.commands.shooter.EjectBall;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.climb.ClimbExtension;
import frc.robot.subsystems.climb.ClimbPassive;
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
                        () -> JoystickContainer.operatingJoystick.getRawAxis(4) > 0.5);

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
                JoystickContainer.BButton.whileActiveContinuous(new IntakeAutomation(0.8));

                JoystickContainer.BButton.whenInactive(new PistonCommand(Intake.getInstance(), false));

                JoystickContainer.AButton.whenActive(new PistonCommand(Intake.getInstance(), true));
                JoystickContainer.AButton.whileActiveContinuous(
                                () -> Conveyor.getInstance().setLowerPower(0.7));
                JoystickContainer.AButton.whileActiveContinuous(
                                () -> Intake.getInstance().setPower(-0.8));
                JoystickContainer.AButton.whenReleased(() -> Intake.getInstance().setPower(0));
                JoystickContainer.AButton.whenReleased(() -> Conveyor.getInstance().setLowerPower(0));
                JoystickContainer.AButton.whenReleased(new PistonCommand(Intake.getInstance(), false));
                JoystickContainer.AButton.whenPressed(new InstantCommand(
                                () -> Conveyor.getInstance()
                                                .setAmountOfBalls(Conveyor.getInstance().getAmountOfBalls() - 1)));
                /*
                JoystickContainer.YButton.whenInactive(() -> Conveyor.getInstance().setUpperPower(0));
                JoystickContainer.YButton.whenInactive(() -> Conveyor.getInstance().setLowerPower(0));
                JoystickContainer.YButton.whileActiveContinuous(() -> Conveyor.getInstance().setLowerPower(-0.7));
                JoystickContainer.YButton.whileActiveContinuous(() -> Conveyor.getInstance().setUpperPower(-0.8));
                JoystickContainer.YButton.whileActiveContinuous(new EjectBall(-1300));
                */
                JoystickContainer.YButton.whileActiveContinuous(new IntakeAutomation(-0.8))
                .whenInactive(new InstantCommand(() -> Intake.getInstance().close()));
                // ---------------------------- Shooter ------------ ----------------
                new Trigger(() -> JoystickContainer.rightJoystick.getRawButton(2))
                                .whileActiveContinuous(new ShooterAutomation());

                new Trigger(() -> JoystickContainer.leftJoystick.getRawButton(1))
                                .whileActiveContinuous(new EjectBall(ShooterConstants.SHOOTER_VELOCITY_FENDER)
                                                .alongWith(new UpperConveyorCommand()));

                new Trigger(() -> JoystickContainer.rightJoystick.getRawButton(3))
                                .whileActiveContinuous(new IntakeAutomation(0.8))
                                .whenInactive(new InstantCommand(() -> Intake.getInstance().close()));

                new Trigger(() -> JoystickContainer.rightJoystick.getRawButton(4)).
                whileActiveContinuous(
                        new SequentialCommandGroup(
                        new PistonCommand(Intake.getInstance(), true),
                        new WaitCommand(0.3),
                        new ParallelDeadlineGroup(
                                new ConveyorCommand(),
                                new MotorCommand(Intake.getInstance(), 0.8)
          ))).whenInactive(new InstantCommand(() -> Intake.getInstance().close()));

                JoystickContainer.XButton.whileActiveContinuous(new EjectBall(1300));
                JoystickContainer.XButton.whileActiveContinuous(() -> Conveyor.getInstance().setUpperPower(-0.9));
                JoystickContainer.XButton.whenReleased(() -> Conveyor.getInstance().setUpperPower(0));

                rightTrigger.whileActiveContinuous(() -> Intake.getInstance().setPower(0));
                rightTrigger.whileActiveContinuous(new EjectBall(-2000));
                rightTrigger.whileActiveContinuous(() -> Conveyor.getInstance().setUpperPower(-0.9));
                rightTrigger.whileActiveContinuous(() -> Conveyor.getInstance().setLowerPower(-0.9));
                rightTrigger.whenInactive(() -> Conveyor.getInstance().setUpperPower(0), Conveyor.getInstance());
                rightTrigger.whenInactive(() -> Conveyor.getInstance().setLowerPower(0), Conveyor.getInstance());
                // ---------------------------- Climb ----------------------------
                JoystickContainer.POVUp
                                .whenActive(new ControlCommand(ClimbExtension.getInstance(),
                                                ClimbConstants.MAX_POSITION, true, true));

                JoystickContainer.POVDown.whenActive(new climbAutomationToSecond());

                JoystickContainer.LB.whileActiveContinuous(new MotorCommand(ClimbPassive.getInstance(), -0.6));

                JoystickContainer.RB.whileActiveContinuous(new MotorCommand(ClimbPassive.getInstance(), 0.6));


                // ---------------------------- Emergency buttons ----------------------------
                // new Trigger(() -> JoystickContainer.rightJoystick.getRawButton(5))
                //                 .whileActiveContinuous(new IntakeAutomation(0.8));

                new Trigger(() -> JoystickContainer.rightJoystick.getRawButton(5))
                                .whenInactive(new PistonCommand(Intake.getInstance(), false));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An ExampleCommand will run in autonomous
                return new BluePathAutonomous();
        }
}
