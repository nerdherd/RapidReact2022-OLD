/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.nerdherd.lib.motor.single.mechanisms.SingleMotorArm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.VisionConstants;

public class Hood extends SingleMotorArm  {
  public double storedAngle = -10;

  public Hood() {
    super(RobotMap.kHoodID, "Hood", false, false);
    // super(RobotMap.kHoodID, "Hood", true, true );
    super.configAngleConversion(HoodConstants.kHoodAngleRatio, HoodConstants.kHoodAngleOffset);
    super.configTrapezoidalConstraints(new TrapezoidProfile.Constraints(HoodConstants.kHoodVel, HoodConstants.kHoodAccel));
    super.configPIDF(HoodConstants.kHoodP, 0, 0, HoodConstants.kHoodF);
    super.configFFs(HoodConstants.kHoodGravityFF, HoodConstants.kHoodStaticFriction);
    super.configOblargConstants(HoodConstants.kHoodS, HoodConstants.kHoodCos, HoodConstants.kHoodV, HoodConstants.kHoodA);
    super.configMotionMagic(HoodConstants.kMotionMagicAcceleration, HoodConstants.kMotionMagicVelocity);
    super.configDeadband(0.0004);
    super.motor.setCoastMode();
    
    
  //96 for entire arm, -28 for start of middle of hood
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    SmartDashboard.putNumber("StoredAngle", storedAngle);
  }

  public void storeAngle(double angle){
    storedAngle = angle;
  }

  public void setStoredAngle(){
    this.setAngleMotionMagic(storedAngle);
  }

  public double distToAngle(double distance){
    double setpointAngle = -10;

    if(distance <= 220 && distance > 169){
      setpointAngle = HoodConstants.kFarPolyA*(Math.pow(distance, 2)) +
                      HoodConstants.kFarPolyB*distance +
                      HoodConstants.kFarPolyC;
    } else if (distance <= 169 && distance > 144){
      setpointAngle = HoodConstants.kFarLinearA*distance +
                      HoodConstants.kFarLinearB;

    } else if (distance <= 144 && distance > 122) {
      setpointAngle = HoodConstants.kCloseLinearA*distance +
                      HoodConstants.kCloseLinearB;
      
    } else if(distance >= 44 && distance < 122){
      setpointAngle = HoodConstants.kClosePolyA*(Math.pow(distance, 2)) +
                      HoodConstants.kClosePolyB*distance +
                      HoodConstants.kClosePolyC;

    } else if (distance > 220){
      setpointAngle = HoodConstants.kTrenchSetPointAngle;
      
    }
    return setpointAngle;
    
  }
}
