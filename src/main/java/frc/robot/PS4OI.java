/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.nerdherd.lib.oi.AbstractOI;

import com.nerdherd.lib.oi.buttons.DPadButton;
import com.nerdherd.lib.oi.buttons.DPadButton.Direction;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.intake.ToggleStow;
import edu.wpi.first.wpilibj2.command.button.Button;

import edu.wpi.first.wpilibj.PS4Controller;

import com.nerdherd.lib.misc.NerdyMath;

// PS4 OI doesn't implement any trigger stuff cause we weren't using it (?)

public class PS4OI extends AbstractOI {

    public PS4Controller driverController;
    public Joystick operatorJoy;
    private double m_joystickDeadband;

    public JoystickButton intakeStow, aimIndex, climbReady, lowGear, highGear, resetEncoder;
    public Button outtakeShift;
    public Button flywheel_RT;

    public final int BUTTON_A = 1, BUTTON_B = 2, BUTTON_X = 3, BUTTON_Y = 4, BUTTON_LB = 5, BUTTON_RB = 6,
            BUTTON_BACK = 7, BUTTON_START = 8, BUTTON_LEFT_STICK = 9, BUTTON_RIGHT_STICK = 10;

    public PS4OI() {
        this(0);
    }

    public PS4OI(double deadband) {
        configJoystickDeadband(deadband);

        driverController = new PS4Controller(0);
        operatorJoy = new Joystick(1);

        intakeStow = new JoystickButton(driverController, BUTTON_LB);
        aimIndex = new JoystickButton(driverController, BUTTON_RB);
        climbReady = new JoystickButton(driverController, BUTTON_X);
        lowGear = new JoystickButton(driverController, BUTTON_A);
        highGear = new JoystickButton(driverController, BUTTON_B);
        resetEncoder = new JoystickButton(driverController, BUTTON_BACK);

        outtakeShift = new DPadButton(driverController, Direction.UP);
        
        intakeStow.whenPressed(new ToggleStow());
    }

    public void update() {
        // update the deadband from smartdashboard
        configJoystickDeadband(SmartDashboard.getNumber("deadband", getJoystickDeadband()));
    }

    public boolean getRawButton(int n) {
        return driverController.getRawButton(n);
    }

    public void setRumble(RumbleType rumbleType, double value) {
        driverController.setRumble(rumbleType, value);
    }

    // lerping was false for Xbox so removed lerping code

    @Override
    public double getDriveJoyLeftX() {
        return NerdyMath.handleDeadband(driverController.getLeftX(), m_joystickDeadband);
    }

    @Override
    public double getDriveJoyLeftY() {
        return NerdyMath.handleDeadband(-driverController.getLeftY(), m_joystickDeadband);
    }

    @Override
    public double getDriveJoyRightX() {
        return NerdyMath.handleDeadband(driverController.getRightX(), m_joystickDeadband);
    }

    @Override
    public double getDriveJoyRightY() {
        return NerdyMath.handleDeadband(-driverController.getRightY(), m_joystickDeadband);
    }

    @Override
    public double getOperatorJoyX() {
        return operatorJoy.getX();
    }

    @Override
    public double getOperatorJoyY() {
        return -operatorJoy.getY();
    } 

    public void configJoystickDeadband(double deadband) {
        m_joystickDeadband = deadband;
    }

    public double getJoystickDeadband() {
        return m_joystickDeadband;
    }

    @Override
    public void initLoggingData() {
        
    }
}
