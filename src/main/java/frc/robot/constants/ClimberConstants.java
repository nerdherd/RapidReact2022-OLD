/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.constants;

/**
 * Add your docs here.
 */
public class ClimberConstants {
    public static final double kArmReadyAngle = 0; // Angle of arm before lifting robot off ground, should go backwards
    public static final double kElevatorReadyPosition = 0;
    public static final double kElevatorLiftPosition = 0;
    public static final double kApproachSpeed = 0; // Speed drivetrain approaches bar
    public static final double kArmHitCurrentChange = 0; // Expected increase in current when arm collides with object
    public static final double kArmRotBack = 0;
    public static final double kArmRotForwards = 0; // Should be slightly more than needed because current detection is also used
    public static final double kArmTravelUp = 0;
    public static final double kArmTravelDown = 0;


    // Robot elevator constants
    public static final double kClimberTalonDeadband = 0.004;
    public static final double kClimberDistanceRatio = 1. / 4096. * 1.432 * Math.PI;
    public static final double kClimberHeightOffset = 2 + 7.5;
    public static final double kClimberDesiredLiftPow = -0.75;
    public static final double kClimberDesiredUpPow = 0;
    public static final double kClimberDesiredHoldPow = -0.1;
    public static final double kHardStopPos = 740000;
    public static final double kClimbGoodPos = 200000;
 
 
    // Arm constants
    public static final double kArmAngleRatio = 0;
    public static final double kArmAngleOffset = 0;
    public static final double kArmVel = 0;
    public static final double kArmAccel = 0;
    public static final double kArmP = 0;
    public static final double kArmF = 0;
    public static final double kArmGravityFF = 0;
    public static final double kArmStaticFriction = 0;
    public static final double kArmS = 0;
    public static final double kArmCos = 0;
    public static final double kArmV = 0;
    public static final double kArmA = 0;
    public static final int kMotionMagicAcceleration = 0;
    public static final int kMotionMagicVelocity = 0;
 
 
    // Arm Elevator constants
    public static final double kArmHoldPos = 0;
    public static final double kArmLowerPos = 0;
    public static final double kArmDesiredLiftPow = 0;
    public static final double kArmDesiredLowPow = 0;
    public static final double kArmDesiredHoldPow = 0;
 
    public static final double kRungAngle = 0.568977336;

    
    // public static final double kClimberTalonDeadband = 0.004;
    // public static final double kClimberDistanceRatio = 1. / 4096. * 1.432 * Math.PI;
    // public static final double kClimberHeightOffset = 2 + 7.5;
    // public static final double kClimberDesiredLiftPow = -0.75;
    // public static final double kClimberDesiredUpPow = 0;
    // public static final double kClimberDesiredHoldPow = -0.1;
    // public static final double kHardStopPos = 740000;
    // public static final double kClimbGoodPos = 200000;

    // public static final double kArmAngleRatio = 0;
    // public static final double kArmAngleOffset = 0;
    // public static final double kArmVel = 0;
    // public static final double kArmAccel = 0;
    // public static final double kArmP = 0;
    // public static final double kArmF = 0;
    // public static final double kArmGravityFF = 0;
    // public static final double kArmStaticFriction = 0;
    // public static final double kArmS = 0;
    // public static final double kArmCos = 0;
    // public static final double kArmV = 0;
    // public static final double kArmA = 0;
    // public static final int kMotionMagicAcceleration = 0;
    // public static final int kMotionMagicVelocity = 0;

    // public static final double kRungAngle = 0.568977336;
}