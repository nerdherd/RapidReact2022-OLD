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
    public static final double kClimberTalonDeadband = 0.004;
    public static final double kClimberDistanceRatio = 1. / 4096. * 1.432 * Math.PI;
    public static final double kClimberHeightOffset = 2 + 7.5;
    public static final double kClimberDesiredLiftPow = -0.75;
    public static final double kClimberDesiredUpPow = 0;
    public static final double kClimberDesiredHoldPow = -0.1;
    public static final double kHardStopPos = 740000;
    public static final double kClimbGoodPos = 200000;
}