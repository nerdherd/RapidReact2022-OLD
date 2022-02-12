/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.constants;
import java.util.*;
import java.lang.*;

/**
 * Add your docs here.
 */

 // everything is in inches 


public class VisionConstants {

    // OLD CONSTANTS!
    public final static double kHorizontalFOV = 55;
    public final static double kVerticalFOV = 42.65386; // calculated from focalLength
    public final static double kHorizontalPixels = 960;
    public final static double kVerticalPixels = 720; 
    public final static double kXFocalLength = 341.3307738; // focalLength = px_width / (2 * tan(FOV / 2))
    public final static double kXFocalLength_lime = 249.68216560510; //709.170231847; 249.68216560510 320*240
    public final static double kYFocalLength = 332.3115843; //NOT ACCURATE?

    // LIMELIGHT CONSTANTS!
    public final static double kHorizontalFOV_lime = 60.22274663;
    public final static double kVerticalFOV_lime = 55.19214793;
    public final static double kRotP_lime = .004;
    public final static double kRotF_lime = 0;

    // Physical camera constants
    public final static double kCameraHorizontalMountAngle = -25.2; 
    public final static double kCameraMountHeight = 23.00; //15.25
    public final static double kCameraHorizontalOffset = 0;
    public final static double kCameraBumperOffset = 16;
    
    public final static double kTargetHeight = 74.25; // inches 74.25 for field 
    public final static double kTargetWidth = 39.25;
    
    // Drive constants

    public final static double kDriveRotationDeadband = 0.5;
    public final static double kDetectDistance = 25;


    public final static double kVecP_lime = 0.004;
    public final static double kShortShot = 0;
    public final static double kLongShot = 0;

}