/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.nerdherd.lib.drivetrain.auto.DriveDistanceMotionMagic;
import com.nerdherd.lib.drivetrain.auto.DriveStraightContinuous;
import com.nerdherd.lib.drivetrain.auto.ResetDriveEncoders;
import com.nerdherd.lib.drivetrain.auto.ResetGyro;
import com.nerdherd.lib.drivetrain.characterization.DriveCharacterizationTest;
import com.nerdherd.lib.drivetrain.shifting.ShiftHigh;
import com.nerdherd.lib.drivetrain.shifting.ShiftLow;
import com.nerdherd.lib.motor.commands.ResetSingleMotorEncoder;
import com.nerdherd.lib.motor.commands.SetMotorPower;
import com.nerdherd.lib.motor.single.SingleMotorMechanism;
import com.nerdherd.lib.oi.DefaultOI;
import com.nerdherd.lib.oi.XboxDriverOI;
import com.nerdherd.lib.oi.buttons.DPadButton;
import com.nerdherd.lib.oi.buttons.DPadButton.Direction;

import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

// import org.graalvm.compiler.lir.aarch64.AArch64Move.StoreConstantOp;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.intake.ToggleStow;
import edu.wpi.first.wpilibj2.command.button.Button;

import com.nerdherd.lib.oi.controllers.NerdyXboxController.Hand;

/**
 * Xbox OI Class
 */
public class XboxOI extends XboxDriverOI {

    public JoystickButton intakeStow, aimIndex, climbReady, lowGear, highGear, resetEncoder;
    public Button outtakeShift;
    public Button flywheel_RT;

    public final int BUTTON_A = 1, BUTTON_B = 2, BUTTON_X = 3, BUTTON_Y = 4, BUTTON_LB = 5, BUTTON_RB = 6,
            BUTTON_BACK = 7, BUTTON_START = 8, BUTTON_LEFT_STICK = 9, BUTTON_RIGHT_STICK = 10;

    private double m_triggerThreshold = 0.25;

    public XboxOI() {
        this(0);
    }

    public XboxOI(double deadband) {
        super(deadband);

        intakeStow = new JoystickButton(super.driverController, BUTTON_LB);
        aimIndex = new JoystickButton(super.driverController, BUTTON_RB);
        climbReady = new JoystickButton(super.driverController, BUTTON_X);
        lowGear = new JoystickButton(super.driverController, BUTTON_A);
        highGear = new JoystickButton(super.driverController, BUTTON_B);
        resetEncoder = new JoystickButton(super.driverController, BUTTON_BACK);
        outtakeShift = new DPadButton(super.driverController, Direction.UP);
        
        flywheel_RT = new Button() {
            @Override
            public boolean get() {
                return getTrigger(Hand.kRight);
            }
        };

        intakeStow.whenPressed(new ToggleStow());
        // aim index
        // climbReady.whenPressed(new )
    }

    public void update() {
        // update the deadband from smartdashboard
        configJoystickDeadband(SmartDashboard.getNumber("deadband", getJoystickDeadband()));

        // update the triggers
        double intake = getTriggerAxis(Hand.kLeft); // variable speed intake
        // use the intake value
    }

    public boolean getRawButton(int n) {
        return driverController.getRawButton(n);
    }

    public double getTriggerAxis(Hand hand) {
        if (hand == Hand.kLeft) {
            return super.driverController.getLeftTriggerAxis();
        } else if (hand == Hand.kRight) {
            return super.driverController.getRightTriggerAxis();
        }
        return 0.0;
    }

    public void setRumble(RumbleType rumbleType, double value) {
        super.driverController.setRumble(rumbleType, value);
    }

    public boolean getTrigger(Hand hand) {
        return getTriggerAxis(hand) >= m_triggerThreshold;
    }
}
