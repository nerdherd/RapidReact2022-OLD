// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.nerdherd.lib.drivetrain.teleop.TankDrive;
import com.nerdherd.lib.motor.commands.ResetSingleMotorEncoder;
import com.nerdherd.lib.motor.single.SingleMotorVictorSPX;
import com.nerdherd.lib.oi.AbstractOI;
import com.nerdherd.lib.oi.controllers.NerdyXboxController.Hand;
import com.nerdherd.lib.pneumatics.Piston;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.Climber;
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
  public static ResetSingleMotorEncoder hoodReset;

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
  public static Climber climber;

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
    climber = new Climber();
    xbox_oi = new XboxOI();

    intakeRoll = new SingleMotorVictorSPX(RobotMap.kIntakeRoll, "Intake Rollers", false);
    intake = new Piston(RobotMap.kIntakePort1, RobotMap.kIntakePort2);
    hoodReset = new ResetSingleMotorEncoder(Robot.hood);

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
  public void testPeriodic() {}

  public static void runResetCommand() {
    drive.resetEncoders();
    drive.resetYaw();
    drive.resetXY();
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
        m_writer.append(csq);
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
        m_writer.append(csq);
      } catch (IOException e) {
        e.printStackTrace();
        writeException = true;
      }
    }
  }
  
}
