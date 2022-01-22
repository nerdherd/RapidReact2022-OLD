/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.nerdherd.lib.drivetrain.experimental.ShiftingDrivetrain;
import com.nerdherd.lib.motor.motorcontrollers.CANMotorController;
import com.nerdherd.lib.motor.motorcontrollers.NerdySparkMax;
import com.nerdherd.lib.motor.motorcontrollers.SmartCANMotorController;
import com.nerdherd.lib.pneumatics.Piston;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.constants.DriveConstants;

public class Drive extends ShiftingDrivetrain {
  /**
   * Creates a new Drive.
   */
  private DifferentialDrivetrainSim m_driveSim;
  private SimDevice m_gyroSim;
  private Field2d m_field;

  private static SmartCANMotorController leftMaster = new NerdySparkMax(RobotMap.kLeftMasterID, MotorType.kBrushed);
  private static SmartCANMotorController rightMaster = new NerdySparkMax(RobotMap.kRightMasterID, MotorType.kBrushed);
  private static CANMotorController[] leftSlaves = new CANMotorController[] { new NerdySparkMax(RobotMap.kLeftFollower1ID, MotorType.kBrushed) };
  private static CANMotorController[] rightSlaves = new CANMotorController[] { new NerdySparkMax(RobotMap.kRightFollower1ID, MotorType.kBrushed) };
  private static Piston shifter = new Piston(RobotMap.  kShifterPort1ID, RobotMap.kShifterPort2ID);

  public Drive() {
      super(leftMaster, rightMaster, leftSlaves, rightSlaves, true, false, shifter, DriveConstants.kTrackWidth);
      
     super.configMaxVelocity(DriveConstants.kMaxVelocity);
     super.configSensorPhase(false, true);
     super.configTicksPerFoot(DriveConstants.kLeftTicksPerFoot, DriveConstants.kRightTicksPerFoot);
     super.configLeftPIDF(DriveConstants.kLeftP, DriveConstants.kLeftI, DriveConstants.kLeftD, DriveConstants.kLeftF);
     super.configRightPIDF(DriveConstants.kRightP, DriveConstants.kRightI, DriveConstants.kRightD, DriveConstants.kRightF);
     super.configStaticFeedforward(DriveConstants.kLeftRamseteS, DriveConstants.kRightRamseteS);
    
     super.m_leftMaster.configCurrentLimitContinuous(50);
     super.m_rightMaster.configCurrentLimitContinuous(50);
     super.m_leftMaster.configCurrentLimitPeak(50);
     super.m_rightMaster.configCurrentLimitPeak(50);

     for (CANMotorController follower : super.m_leftSlaves) {
       follower.configCurrentLimitContinuous(50);
     }

     for (CANMotorController follower : super.m_leftSlaves) {
      follower.configCurrentLimitPeak(50);
      }

     setCoastMode();
     resetEncoders();
     
     m_field = new Field2d();
     SmartDashboard.putData("Field", m_field);

     if(RobotBase.isSimulation()){
      m_driveSim = new DifferentialDrivetrainSim(
        LinearSystemId.identifyDrivetrainSystem(
          DriveConstants.kVLinear, DriveConstants.kALinear, DriveConstants.kVAngular, DriveConstants.kAAngular), 
        DCMotor.getFalcon500(2), 
        DriveConstants.gearReduction, 
        DriveConstants.kTrackWidth, 
        DriveConstants.wheelRadiusMeters, 
        VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
      
        m_gyroSim = SimDevice.create("navX-Sensor", 0);
     }
  }

  @Override
  public void simulationPeriodic() {
    m_driveSim.setPose(new Pose2d(0.762, 0.762, new Rotation2d(0))); // Slalom
    m_driveSim.setInputs(simLeftVolt, simRightVolt);
    m_driveSim.update(0.02);
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(-m_driveSim.getHeading().getDegrees());

    SmartDashboard.putNumber("Sim Pose X Meters", m_driveSim.getPose().getX());
    SmartDashboard.putNumber("Sim Pose Y Meters", m_driveSim.getPose().getY());
    SmartDashboard.putNumber("Sim heading", m_driveSim.getPose().getRotation().getDegrees());
    m_field.setRobotPose(m_driveSim.getPose());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.updateOdometry();
    super.reportToSmartDashboard();
  }
}

