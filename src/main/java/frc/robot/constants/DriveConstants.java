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
public class DriveConstants {
    public static final double kLeftP = 0.1097 * 2000 / 1856 * 1.73 * 1.143; //replace these
	public static final double kLeftI = 0;
	public static final double kLeftD = 0;

	public static final double kRightP = 0.1097 * 2000 / 1856 * 1.73 * 1.143; //2.12 3/31 for talon
	public static final double kRightI = 0;
    public static final double kRightD = 0;
    
    public static final double kDriveMaxVel = 2.0;
    public static final double kDriveMaxAccel = 1.0;
    
    public static final double kStartAngle = 1.055285;
    public static final double kTarmacToBallOneAngle = 1.257627;

    public static final double kLeftRamseteS = 0.7;
    public static final double kLeftRamseteV = 2.08;
    public static final double kLeftRamseteA = 0.45;

    public static final double kWallOffset = 0.3048;


    public static final double kramseteS = 0.731; //replace these when analyzing middle? 
    public static final double kramseteV = 2.08;
    public static final double kramseteA = 0.429;

    public static final double kRightRamseteS = 0.705;
    public static final double kRightRamseteV = 2.08;
    public static final double kRightRamseteA = 0.43;

	public static final double kRamseteMaxVolts = 10;
    
    public static double kLeftStatic = 0.39; 
    public static double kRightStatic =  0.194;

    public static double kLeftF = 0.005648; //0.005648
    public static double kRightF = 0.005661; //0.005661

    public static int kMaxVelocity = 10000;
    public static double kLeftTicksPerFoot = (11508 + 11969) / 2;
    public static double kRightTicksPerFoot = (11508 + 11969) / 2;

    public static double kTrackWidth = 0.678;
	public static final double kMaxCentripetalAcceleration = 2.1;
    
    public static final double kVLinear = 0.461;
    public static final double kALinear = 0.0767;
    public static final double kVAngular = 0.482;
    public static final double kAAngular = 0.0637;
    
    public static final double gearReduction = 9.6; //8
    public static final double wheelRadiusMeters = 0.152;
    
}