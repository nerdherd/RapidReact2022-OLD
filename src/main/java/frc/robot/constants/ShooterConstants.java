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
public class ShooterConstants {
    public static double kP = 0.18;
    public static int kI = 0;
    public static int kD = 0;
    public static double kF= 0.0477003133;
    public static double kS = 0;
    public static double kV = 0;
    public static double kA = 0;
    public static int kDeadband = 0;

    public static double kVelocityTolerance = 500;
    public static double kVelPercentTolerance = 0.05;


    public static final double kAutolineShotVelocity = 16200;
    public static final double kTrenchShotVelocity = 17200;
    public static final double kWallShotVelocity = 9001;
    public static final double kRendezvousShotVelocity = 18250;
    public static final double kAutoAngleCloseVelocity = 16200;
    public static final double kAutoAngleFarVelocity = 0;
    //6750 Wall shot speed
}
