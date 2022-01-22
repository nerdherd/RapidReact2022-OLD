/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.nerdherd.lib.motor.motorcontrollers.NerdyFalcon;
import com.nerdherd.lib.motor.single.mechanisms.SingleMotorElevator;

import frc.robot.RobotMap;
import frc.robot.constants.ClimberConstants;

/**
 * Add your docs here.
 */
public class Climber extends SingleMotorElevator {
  public static NerdyFalcon followerFalcon = new NerdyFalcon(RobotMap.kClimberID2);
  public static NerdyFalcon mainFalcon = new NerdyFalcon(RobotMap.kClimberID1);

public Climber(){
  super(mainFalcon.getID(), "Climber ", false, false);
  followerFalcon.follow(mainFalcon);
  // mainFalcon.setBrakeMode();
  // followerFalcon.setBrakeMode();
  followerFalcon.setInverted(TalonFXInvertType.OpposeMaster);
  super.configDeadband(ClimberConstants.kClimberTalonDeadband);
  super.configHeightConversion(ClimberConstants.kClimberDistanceRatio,
    ClimberConstants.kClimberHeightOffset);
  super.configCurrentLimit(60, 60);
}

  @Override
  public void reportToSmartDashboard(){
    super.reportToSmartDashboard();
  }


}
