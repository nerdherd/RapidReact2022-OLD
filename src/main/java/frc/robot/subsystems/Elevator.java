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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.constants.ClimberConstants;

/**
 * Add your docs here.
 */
public class Elevator extends SingleMotorElevator {
  public static NerdyFalcon mainFalcon = new NerdyFalcon(RobotMap.kArmID2);

public Elevator(){
  super(mainFalcon.getID(), "Elevator ", true, false);
  // mainFalcon.setBrakeMode();
  // followerFalcon.setBrakeMode();
  super.configDeadband(ClimberConstants.kClimberTalonDeadband);
  super.configHeightConversion(ClimberConstants.kClimberDistanceRatio,
    ClimberConstants.kClimberHeightOffset);
  super.configCurrentLimit(60, 60);
}


public void setHeight(String action, double pos){
  if (action.equals("lift")){
  mainFalcon.setPower(ClimberConstants.kArmDesiredLiftPow);
        if(mainFalcon.getPosition() > pos){
            mainFalcon.setPower(ClimberConstants.kArmDesiredHoldPow);
        }else{
            mainFalcon.setPower(ClimberConstants.kArmDesiredLiftPow);
        }
      }
  else{
    mainFalcon.setPower(ClimberConstants.kArmDesiredLowPow);
    if(mainFalcon.getPosition() < pos){
      mainFalcon.setPower(ClimberConstants.kArmDesiredHoldPow);
    }else{
      mainFalcon.setPower(ClimberConstants.kArmDesiredLowPow);
    }
  }
}

  @Override
  public void reportToSmartDashboard(){
    super.reportToSmartDashboard();
    SmartDashboard.putNumber("ArmElevCur", mainFalcon.getCurrent());
    SmartDashboard.putNumber("ArmElevPos", mainFalcon.getPosition());
  }


}
