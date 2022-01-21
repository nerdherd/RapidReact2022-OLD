/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.nerdherd.lib.motor.dual.DualMotorIntake;
import com.nerdherd.lib.motor.motorcontrollers.NerdySparkMax;
import com.nerdherd.lib.motor.motorcontrollers.NerdyVictorSPX;
import com.nerdherd.lib.motor.single.SingleMotorVictorSPX;


import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Hopper extends DualMotorIntake {

    public static SingleMotorVictorSPX leftRoller = new SingleMotorVictorSPX(RobotMap.kFeederID1, "Left Intake", true);
    public static SingleMotorVictorSPX rightRoller = new SingleMotorVictorSPX(RobotMap.kFeederID2, "Right Intake", false);
    public static NerdyVictorSPX topRoller = new NerdyVictorSPX(RobotMap.kTopHopperRollerID);

    public Hopper() {
        super(leftRoller,rightRoller);
        topRoller.follow(rightRoller.motor);
    }

    @Override
    public void setPower(double leftPower, double rightPower) {
        // TODO Auto-generated method stub
        super.setPower(leftPower, rightPower);
        topRoller.setPower(rightPower);
    }

    public void setPowerWithoutTop(double leftPower, double rightPower) {
        super.setPower(leftPower, rightPower);
    }

    public void setTopHopperPower(double power) {
        topRoller.setPower(power);
    }

}
