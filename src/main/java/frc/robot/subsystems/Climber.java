<<<<<<< HEAD:src/main/java/frc/robot/subsystems/Climber.java
=======
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
>>>>>>> master:src/main/java/frc/robot/subsystems/Arm.java
package frc.robot.subsystems;
 
import com.nerdherd.lib.motor.single.mechanisms.SingleMotorArm;
<<<<<<< HEAD:src/main/java/frc/robot/subsystems/Climber.java
=======
 
>>>>>>> master:src/main/java/frc/robot/subsystems/Arm.java
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.nerdherd.lib.motor.motorcontrollers.NerdyFalcon;
import com.nerdherd.lib.motor.single.mechanisms.SingleMotorElevator;

import frc.robot.RobotMap;
import frc.robot.constants.ClimberConstants;
<<<<<<< HEAD:src/main/java/frc/robot/subsystems/Climber.java

public class Climber {
    public class Arm extends SingleMotorArm {
        public double storedAngle = 0;

        public Arm() {
        super(RobotMap.kArmID, "Arm", false, false);
        super.configAngleConversion(ClimberConstants.kArmAngleRatio, ClimberConstants.kArmAngleOffset);
        super.configTrapezoidalConstraints(new TrapezoidProfile.Constraints(ClimberConstants.kArmVel, ClimberConstants.kArmAccel));
        super.configPIDF(ClimberConstants.kArmP, 0, 0, ClimberConstants.kArmF);
        super.configFFs(ClimberConstants.kArmGravityFF, ClimberConstants.kArmStaticFriction);
        super.configOblargConstants(ClimberConstants.kArmS, ClimberConstants.kArmCos, ClimberConstants.kArmV, ClimberConstants.kArmA);
        super.configMotionMagic(ClimberConstants.kMotionMagicAcceleration, ClimberConstants.kMotionMagicVelocity);
        super.configDeadband(0.0004);
        super.motor.setCoastMode();
        }

        @Override
        public void periodic() {
            // This method will be called once per scheduler run
        }

        public void storeAngle(double angle){
            storedAngle = angle;
        }

        public void setStoredAngle(){
            this.setAngleMotionMagic(storedAngle);
        }
    }    

    public static class Elevator extends SingleMotorElevator {
        public static NerdyFalcon mainFalcon = new NerdyFalcon(RobotMap.kElevatorID);

        public Elevator(){
            super(mainFalcon.getID(), "Climber ", true, false);
            // mainFalcon.setBrakeMode();
            // followerFalcon.setBrakeMode();
            super.configDeadband(ClimberConstants.kClimberTalonDeadband);
            super.configHeightConversion(ClimberConstants.kClimberDistanceRatio,
            ClimberConstants.kClimberHeightOffset);
            super.configCurrentLimit(60, 60);
        }

        @Override
        public void reportToSmartDashboard() {
            super.reportToSmartDashboard();
        }
    }
=======
 
public class Arm extends SingleMotorArm {
 
  public double storedAngle = 0;
 
  /** Creates a new Arm. */
  public Arm() {
    super(RobotMap.kArmID1, "Arm", false, false);
    super.configAngleConversion(ClimberConstants.kArmAngleRatio, ClimberConstants.kArmAngleOffset);
    super.configTrapezoidalConstraints(new TrapezoidProfile.Constraints(ClimberConstants.kArmVel, ClimberConstants.kArmAccel));
    super.configPIDF(ClimberConstants.kArmP, 0, 0, ClimberConstants.kArmF);
    super.configFFs(ClimberConstants.kArmGravityFF, ClimberConstants.kArmStaticFriction);
    super.configOblargConstants(ClimberConstants.kArmS, ClimberConstants.kArmCos, ClimberConstants.kArmV, ClimberConstants.kArmA);
    super.configMotionMagic(ClimberConstants.kMotionMagicAcceleration, ClimberConstants.kMotionMagicVelocity);
    super.configDeadband(0.0004);
    super.motor.setCoastMode();
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ArmAngle", this.getAngle());
  }
 
  public void storeAngle(double angle){
    storedAngle = angle;
  }
 
  public void setStoredAngle(){
    this.setAngleMotionMagic(storedAngle);
  }
>>>>>>> master:src/main/java/frc/robot/subsystems/Arm.java
}
