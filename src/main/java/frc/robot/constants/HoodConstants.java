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
public class HoodConstants {

    public static final double kHoodGravityFF = 0.0671;
    // difficult to get data going downward because there is no tension
    public static final double kHoodStaticFriction = 0.3;// 1.263;
    public static final int kMotionMagicAcceleration = 2500;
    public static final int kMotionMagicVelocity = 2500;
    public static final double kHoodP = 1;
    public static final double kHoodF = 0.369644;
    // 0.1092 kHoodF OG
    public static final double kHoodAngleRatio = 360./4096./15.;
    public static final double kHoodAngleOffset = -10;
    
    public static final double kHoodAccel = 116;
    public static final double kHoodVel = 168;

    public static final double kHoodV = 0.0715;
    public static final double kHoodA = 0.00103;
    public static final double kHoodS = 0.403;
    public static final double kHoodCos = 0.0671;

    public static final double kWallShotAngle = 0;
    public static final double kTrenchShotAngle = 31;
    public static final double kAutolineShotAngle = 26;
    public static final double kRendezvousShotAngle = 29;
    public static final double kStowAngle = -10;

    // POLYNOMIAL REGRESSION - FAR
    public static final double kFarPolyA = -7.99065708e-04;
    public static final double kFarPolyB = 3.79464011e-01;
    public static final double kFarPolyC = -1.08073022e+01;

    // POLYNOMIAL REGRESSION - CLOSE
    public static final double kClosePolyA = -2.40032788e-03;
    public static final double kClosePolyB = 5.96478736e-01;
    public static final double kClosePolyC = -1.04487478e+01;

    // LINEAR REGRESSION - FAR
    public static final double kFarLinearA = 2.000e-02;
    public static final double kFarLinearB = 2.712e+01;

    // LINEAR REGRESSION - CLOSE
    public static final double kCloseLinearA = 0.15909091;
    public static final double kCloseLinearB = 7.09090909;

    //WHEN WE'RE DONE W THE TRENCH WOOT WOOT

    public static final double kTrenchSetPointAngle = 30.75;
    
    // LINEAR REGRESSION - TRENCH 
    public static final double kTrenchLinearA = 0.15909091; // need to chANGE
    public static final double kTrenchLinearB = 7.09090909; // look up 


}
