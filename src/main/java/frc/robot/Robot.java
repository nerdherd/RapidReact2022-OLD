// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.nerdherd.lib.drivetrain.teleop.TankDrive;
import com.nerdherd.lib.motor.commands.ResetSingleMotorEncoder;
import com.nerdherd.lib.motor.single.SingleMotorVictorSPX;
import com.nerdherd.lib.motor.motorcontrollers.NerdySparkMax;
import com.nerdherd.lib.motor.motorcontrollers.NerdyTalon;
import com.nerdherd.lib.motor.motorcontrollers.NerdyFalcon;
import com.nerdherd.lib.oi.AbstractOI;
import com.nerdherd.lib.oi.controllers.NerdyXboxController.Hand;
import com.nerdherd.lib.pneumatics.Piston;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmElevator;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Jevois;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static final String kDate = "2021_1_15_";
  public static SingleMotorVictorSPX intakeRoll;
  private String m_filePath1 = "/media/sda1/logs/";
  private String m_filePath2 = "/home/lvuser/logs/";
  private File m_file;
  public FileWriter m_writer;
  private boolean writeException = false;
  private double m_logStartTime = 0;

  private static AHRS m_nav;

  public static ResetSingleMotorEncoder hoodReset;

  public static NerdySparkMax leftMaster;
  public static NerdySparkMax rightMaster;
  public static NerdySparkMax leftFollower;
  public static NerdySparkMax rightFollower;

  public static Arm arm;

  public static XboxOI xbox_oi;
  public static Drive drive;
  public static Jevois jevois;
  public static Limelight limelight;
  public static DriverStation ds;
  public static Indexer indexer;
  public static Hopper hopper;
  public static Hood hood;
  public static Piston intake;
  public static Shooter shooter;
  public static Arm arm;
  public static ArmElevator armElev;
  public static Elevator elevator;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    drive = new Drive();
    indexer = new Indexer();
    hopper = new Hopper();
    hood = new Hood();
    shooter = new Shooter();
    arm = new Arm();
    armElev = new ArmElevator();
    elevator = new Elevator();
    xbox_oi = new XboxOI();
    

    intakeRoll = new SingleMotorVictorSPX(RobotMap.kIntakeRoll, "Intake Rollers", false);
    intake = new Piston(RobotMap.kIntakePort1, RobotMap.kIntakePort2);
    hoodReset = new ResetSingleMotorEncoder(Robot.hood);
    leftMaster = new NerdySparkMax(RobotMap.kLeftMasterID, MotorType.kBrushless);
    rightMaster = new NerdySparkMax(RobotMap.kRightMasterID, MotorType.kBrushless);
    leftFollower = new NerdySparkMax(RobotMap.kLeftFollower1ID, MotorType.kBrushless);
    rightFollower = new NerdySparkMax(RobotMap.kRightFollower1ID, MotorType.kBrushless);
    
    elevator = new NerdyFalcon(RobotMap.kClimberID1);
    arm = new NerdyTalon(RobotMap.kArmID1);

    drive.setDefaultCommand(new TankDrive(Robot.drive, Robot.xbox_oi));
    drive.configKinematics(DriveConstants.kTrackWidth, new Rotation2d(0), new Pose2d(0, 0, new Rotation2d(0)));
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    drive.reportToSmartDashboard();
    shooter.reportToSmartDashboard();
    hopper.reportToSmartDashboard();
    indexer.reportToSmartDashboard();
    limelight.reportToSmartDashboard();
    hood.reportToSmartDashboard();
    arm.reportToSmartDashboard();
    armElev.reportToSmartDashboard();
    elevator.reportToSmartDashboard();
    
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    hood.resetEncoder();
    drive.setCoastMode();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    drive.setBrakeMode();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    drive.setCoastMode();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
    xbox_oi.update(); // update the oi
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    SmartDashboard.putNumber("LMVolt", getLeftMasterVel());
  }

  public static void runResetCommand() {
    drive.resetEncoders();
    drive.resetYaw();
    drive.resetXY();
  }

  public void reportToSmartDashboard() {
    SmartDashboard.putNumber("LMVoltage", getLeftMasterVoltage());
    SmartDashboard.putNumber("LMCurrent", getLeftMasterCurrent());
    SmartDashboard.putNumber("LMVelocity", getLeftMasterVel());
    SmartDashboard.putNumber("LMPosition", getLeftMasterPos());
    SmartDashboard.putNumber("RMVoltage", getRightMasterVoltage());
    SmartDashboard.putNumber("RMCurrent", getRightMasterCurrent());
    SmartDashboard.putNumber("RMVelocity", getRightMasterVel());
    SmartDashboard.putNumber("RMPosition", getRightMasterPos());
    SmartDashboard.putNumber("LFVoltage", getLeftFollowerVoltage());
    SmartDashboard.putNumber("LFCurrent", getLeftFollowerCurrent());
    SmartDashboard.putNumber("LFVelocity", getLeftFollowerVel());
    SmartDashboard.putNumber("LFPosition", getLeftFollowerPos());
    SmartDashboard.putNumber("RFVoltage", getRightFollowerVoltage());
    SmartDashboard.putNumber("RFCurrent", getRightFollowerCurrent());
    SmartDashboard.putNumber("RFVelocity", getRightFollowerVel());
    SmartDashboard.putNumber("RFPosition", getRightFollowerPos());
    SmartDashboard.putNumber("EVoltage", getElevatorVoltage());
    SmartDashboard.putNumber("ECurrent", getElevatorCurrent());
    SmartDashboard.putNumber("EVelocity", getElevatorVel());
    SmartDashboard.putNumber("EPosition", getElevatorPos());
    SmartDashboard.putNumber("AVoltage", getArmVoltage());
    SmartDashboard.putNumber("ACurrent", getArmCurrent());
    SmartDashboard.putNumber("AVelocity", getArmVel());
    SmartDashboard.putNumber("APosition", getArmPos());
    SmartDashboard.putNumber("CRoll", getCurrentRoll());
    SmartDashboard.putNumber("CPitch", getCurrentPitch());
    SmartDashboard.putNumber("CYaw", getCurrentYaw());
    SmartDashboard.putNumber("NavTS", getNavTimestamp());
    SmartDashboard.putNumber("CAccelX", getCurrentAccelX());
    SmartDashboard.putNumber("CAccelY", getCurrentAccelY());
    SmartDashboard.putNumber("CAccelZ", getCurrentAccelZ ());

  }

  public static void resetGyro() {
    m_nav.reset();
  }

  public double getLeftMasterVoltage() {
    return leftMaster.getVoltage();
  }

  public double getLeftMasterCurrent() {
    return leftMaster.getCurrent();
  }

  public double getLeftMasterVel() {
    return leftMaster.getVelocity();
  }

  public double getLeftMasterPos() {
    return leftMaster.getPosition();
  }

  public double getRightMasterVoltage() {
    return rightMaster.getVoltage();
  }

  public double getRightMasterCurrent() {
    return rightMaster.getCurrent();
  }

  public double getRightMasterVel() {
    return rightMaster.getVelocity();
  }

  public double getRightMasterPos() {
    return rightMaster.getPosition();
  }

  public double getLeftFollowerVoltage() {
    return leftFollower.getVoltage();
  }

  public double getLeftFollowerCurrent() {
    return leftFollower.getCurrent();
  }

  public double getLeftFollowerVel() {
    return leftFollower.getVelocity();
  }

  public double getLeftFollowerPos() {
    return leftFollower.getPosition();
  }

  public double getRightFollowerVoltage() {
    return rightFollower.getVoltage();
  }

  public double getRightFollowerCurrent() {
    return rightFollower.getCurrent();
  }

  public double getRightFollowerVel() {
    return rightFollower.getVelocity();
  }

  public double getRightFollowerPos() {
    return rightFollower.getPosition();
  }

  public double getElevatorVoltage() {
    return elevator.getVoltage();
  }

  public double getElevatorCurrent() {
    return elevator.getCurrent();
  }

  public double getElevatorVel() {
    return elevator.getVelocity();
  }

  public double getElevatorPos() {
    return elevator.getPosition();
  }

  public double getArmVoltage() {
    return arm.getVoltage();
  }

  public double getArmCurrent() {
    return arm.getCurrent();
  }

  public double getArmVel() {
    return arm.getVelocity();
  }

  public double getArmPos() {
    return arm.getPosition();
  }

  public double getCurrentRoll() {
    return m_nav.getRoll();
  }

  public double getCurrentPitch() {
    return m_nav.getPitch();
  }

  public double getCurrentYaw() {
    return m_nav.getYaw();
  }

  public double getNavTimestamp() {
    	return m_nav.getLastSensorTimestamp();
  }

  public double getCurrentAccelX() {
	    return m_nav.getWorldLinearAccelX();
  }

  public double getCurrentAccelY() {
    return m_nav.getWorldLinearAccelY();
  }

  public double getCurrentAccelZ() {
    return m_nav.getWorldLinearAccelZ();
  }

  public void startLog(){
    File logFolder1 = new File(m_filePath1);
	  File logFolder2 = new File(m_filePath2);
    Path filePrefix = Paths.get("");
    if (logFolder1.exists() && logFolder1.isDirectory()) {
      filePrefix = Paths.get(logFolder1.toString(), 
      Robot.kDate + Robot.ds.getMatchType().toString() + Robot.ds.getMatchNumber() + "Drive");
    } else if (logFolder2.exists() && logFolder2.isDirectory()) {
      filePrefix = Paths.get(logFolder2.toString(), 
      Robot.kDate + Robot.ds.getMatchType().toString() + Robot.ds.getMatchNumber() + "Drive");
    } else {
      writeException = true;
    }

    if (!writeException) {
      int counter = 0;
      while (counter <= 99) {
        m_file = new File(filePrefix.toString() + String.format("%02d", counter) + ".csv");
        if (m_file.exists()) {
          counter++;
        } else {
          break;
        }
        if (counter == 99) {
          System.out.println("Log creation counter at 99.");
        }
      }
      try {
        m_writer = new FileWriter(m_file);
        m_writer.append("Time, Match Time, Pitch, Roll, Yaw" + "X Accel, Y Accel, Z Accel" + 
                "Left Master Voltage, Left Master Current, Right Master Voltage, Right Master Current" +
                "Left Follower Voltage, Left Follower Current, Right Follower Current, Right Follower Current" +
                "Left Master Vel, Left Master Pos, Right Master Vel, Right Master Pos" +
                "Left Follower Vel, Left Follower Pos, Right Follower Vel, Right Follower Pos" +
                "Elevator Voltage, Elevator Current, Elevator Velocity, Elevator Position" + 
                "Arm Voltage, Arm Current, Arm Velocity, Arm Position/n");
        m_logStartTime = Timer.getFPGATimestamp();
      } catch (IOException e) {
        e.printStackTrace();
		    writeException = true;
      }
    }
  }

  public void stopLog() {
    try {
	    if (null != m_writer) {
		    m_writer.close();
      }
	  } catch (IOException e) {
	      e.printStackTrace();
	      writeException = true;
	  }
  }

  public void logToCSV(){
    System.out.println("Test write");
    if (!writeException) {
      try {
        double timestamp = Timer.getFPGATimestamp() - m_logStartTime;
        m_writer.append(String.valueOf(timestamp) + "," + String.valueOf(Robot.ds.getMatchTime()) + "," 
                + String.valueOf(getCurrentPitch()) + "," + String.valueOf(getCurrentRoll()) + ","
                + String.valueOf(getCurrentYaw()) + "," + String.valueOf(getCurrentAccelX()) + ","
                + String.valueOf(getCurrentAccelY()) + "," + String.valueOf(getCurrentAccelZ()) + ","
                + String.valueOf(getLeftMasterVoltage()) + "," + String.valueOf(getLeftMasterCurrent()) + ","
                + String.valueOf(getRightMasterVoltage()) + "," + String.valueOf(getRightMasterCurrent()) + ","
                + String.valueOf(getLeftFollowerVoltage()) + "," + String.valueOf(getLeftFollowerCurrent()) + ","
                + String.valueOf(getRightFollowerVoltage()) + "," + String.valueOf(getRightFollowerCurrent()) + ","
                + String.valueOf(getLeftMasterVel()) + "," + String.valueOf(getLeftMasterPos()) + ","
                + String.valueOf(getRightMasterVel()) + "," + String.valueOf(getRightMasterPos()) + ","
                + String.valueOf(getLeftFollowerVel()) + "," + String.valueOf(getLeftFollowerPos()) + ","
                + String.valueOf(getRightFollowerVel()) + "," + String.valueOf(getRightFollowerPos()) + ","
                + String.valueOf(getElevatorVoltage()) + "," + String.valueOf(getElevatorCurrent()) + ","
                + String.valueOf(getElevatorVel()) + "," + String.valueOf(getElevatorPos()) + ","
                + String.valueOf(getArmVoltage()) + "," + String.valueOf(getArmCurrent()) + ","
                + String.valueOf(getArmVel()) + "," + String.valueOf(getArmPos()) + ",");
      } catch (IOException e) {
        e.printStackTrace();
        writeException = true;
      }
    }
  }

}
