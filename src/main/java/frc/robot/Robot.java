// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

/**
 * This is a demo program showing the use of the DifferentialDrive class,
 * specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
    private static final Timer ROBOT_TIMER = new Timer();
    private final DifferentialDrive robotDrive;
    private final Joystick leftStick;
    private final Joystick rightStick;
    // TODO: You might also use an XboxController instead of a Joystick.

    // TODO: Remember to verify your motor controller type and port numbers/CAN IDs
    private final PWMSparkMax leftMotors = new PWMSparkMax(0);
    private final PWMSparkMax rightMotors = new PWMSparkMax(1);
    private final PWMSparkMax coralMotor = new PWMSparkMax(2);

    private boolean wasAutonExecuted;
    private int autonStep;

    /** Called once at the beginning of the robot program. */
    public Robot() {
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        rightMotors.setInverted(true);

        robotDrive = new DifferentialDrive(leftMotors::set, rightMotors::set);
        leftStick = new Joystick(0);
        rightStick = new Joystick(1);

        // TODO: Remember to add your motors to the SendableRegistry.
        SendableRegistry.addChild(robotDrive, leftMotors);
        SendableRegistry.addChild(robotDrive, rightMotors);
        SendableRegistry.add(coralMotor, "Coral Motor");
    }

    @Override
    public void disabledInit() {
        ROBOT_TIMER.stop();
        ROBOT_TIMER.reset();
    }

    @Override
    public void teleopPeriodic() {
        double coralSpeed = 0.0; // Default to no movement
        // Get the buttons from a Joystick or XboxController and use them
        // to control the coral mechanism.
        if (leftStick.getRawButton(1) || rightStick.getRawButton(1)) {
            coralSpeed = 0.5;
        } else if (leftStick.getRawButton(2) || rightStick.getRawButton(2)) {
            coralSpeed = -0.5;
        }

        // Set the motor outputs.
        robotDrive.tankDrive(-leftStick.getY(), -rightStick.getY());
        coralMotor.set(coralSpeed);
    }

    @Override
    public void autonomousInit() {
        wasAutonExecuted = false;
        autonStep = 0;
    }

    @Override
    public void autonomousPeriodic() {
        // TODO: This auton routine is just an example. You will need to test and
        // adjust it for your robot and strategy.
        if (wasAutonExecuted) {
            // Auton has already been executed, so do nothing.
            robotDrive.tankDrive(0.0, 0.0);
            coralMotor.set(0.0);
            return;
        }
        // Auton has not been executed yet, so run the auton routine.
        switch (autonStep) {
            case 0:
                // Start the timer if it hasn't been started yet.
                ROBOT_TIMER.start();
                // Drive forward for 2 seconds.
                coralMotor.set(0.0);
                robotDrive.tankDrive(0.5, 0.5);
                if (ROBOT_TIMER.get() > 2.0) {
                    robotDrive.tankDrive(0.0, 0.0);
                    autonStep++;
                }
                break;
            case 1:
                // Turn right for 1 second.
                coralMotor.set(0.0);
                robotDrive.tankDrive(0.5, -0.5);
                if (ROBOT_TIMER.get() > 3.0) {
                    robotDrive.tankDrive(0.0, 0.0);
                    autonStep++;
                }
                break;
            case 2:
                // Run coral mechanism for 2 seconds.
                coralMotor.set(0.5);
                robotDrive.tankDrive(0.0, 0.0);
                if (ROBOT_TIMER.get() > 5.0) {
                    coralMotor.set(0.0);
                    autonStep++;
                }
                break;
            default:
                coralMotor.set(0.0);
                robotDrive.tankDrive(0.0, 0.0);
                wasAutonExecuted = true;
                ROBOT_TIMER.stop();
                break;
        }
    }
}
