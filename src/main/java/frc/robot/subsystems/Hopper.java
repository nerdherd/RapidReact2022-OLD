/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.nerdherd.lib.motor.dual.DualMotorIntake;
import com.nerdherd.lib.motor.motorcontrollers.NerdyVictorSPX;
import com.nerdherd.lib.motor.single.SingleMotorVictorSPX;

import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Hopper extends DualMotorIntake {

    public static NerdyVictorSPX topRoller;
    static SingleMotorVictorSPX leftRoller = new SingleMotorVictorSPX(RobotMap.kFeederID1, "Top Intake", true);

    public Hopper() {
        super(leftRoller, 
            new SingleMotorVictorSPX(RobotMap.kFeederID2, "Bottom Intake", false));
        topRoller = new NerdyVictorSPX(RobotMap.kTopHopperRollerID);
        topRoller.follow(leftRoller.motor);
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
