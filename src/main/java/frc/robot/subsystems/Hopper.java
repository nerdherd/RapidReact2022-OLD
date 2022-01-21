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
import com.nerdherd.lib.motor.single.SingleMotorMechanism;
import com.nerdherd.lib.motor.single.SingleMotorVictorSPX;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Hopper extends DualMotorIntake {

    public static NerdySparkMax leftMotorControl = new NerdySparkMax(RobotMap.kFeederID1, MotorType.kBrushed);
    public static NerdySparkMax rightMotorControl = new NerdySparkMax(RobotMap.kFeederID2, MotorType.kBrushed);
    
    public static SingleMotorMechanism leftRoller = new SingleMotorMechanism(
        leftMotorControl, "Left Intake", true, RobotMap.kSensorPhase1);
    public static SingleMotorMechanism rightRoller = new SingleMotorMechanism(
        rightMotorControl, "Right Intake", true, RobotMap.kSensorPhase2);
    public static NerdySparkMax topRoller = new NerdySparkMax(RobotMap.kTopHopperRollerID, MotorType.kBrushed);
    

    public Hopper() {
        super(leftRoller,rightRoller);
        topRoller.follow(rightMotorControl);
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
