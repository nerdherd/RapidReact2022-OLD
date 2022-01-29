/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.List;

import com.nerdherd.lib.drivetrain.characterization.OpenLoopDrive;
import com.nerdherd.lib.drivetrain.experimental.Drivetrain;

import frc.robot.Robot;
import frc.robot.commands.vision.MoveToAngleLime;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.IntakeBalls;
import frc.robot.commands.shooting.ShootBall;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.VisionConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TarmacToTerminalFive extends SequentialCommandGroup {
    
    private Drivetrain m_drive;

    public TarmacToTerminalFive(Drivetrain drive) {
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveConstants.kramseteS, DriveConstants.kramseteV, DriveConstants.kramseteA),
        m_drive.m_kinematics, 
        DriveConstants.kRamseteMaxVolts);

        var autoCentripetalAccelerationConstraint = new CentripetalAccelerationConstraint(DriveConstants.kMaxCentripetalAcceleration);
    
        TrajectoryConfig config = new TrajectoryConfig(DriveConstants.kDriveMaxVel, DriveConstants.kDriveMaxAccel);
        config.addConstraints(List.of(autoVoltageConstraint, autoCentripetalAccelerationConstraint));

        Trajectory tarmacToLeftBall = TrajectoryGenerator.generateTrajectory( // Backs up after intaking left ball 
            new Pose2d(0.616995, 2.320925, new Rotation2d(0)),
            List.of(new Translation2d(0.757540, 3.811651 - DriveConstants.kWallOffset)),
            new Pose2d(0.616995, 1.02235, new Rotation2d(0)),
            config);

        Trajectory leftBallToMidBall = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0.616995, 1.02235, new Rotation2d(0)),
            new ArrayList<Translation2d>(),
            new Pose2d(6.72583252494, 3.22479779474, new Rotation2d(0.763581548)),
            config);
            
        Trajectory midBallToTerminal = TrajectoryGenerator.generateTrajectory(
            new Pose2d(3.1736324132, 2.24290163814, new Rotation2d(0.615228561)),
            new ArrayList<Translation2d>(),
            new Pose2d(6.72583252494, 3.22479779474, new Rotation2d(0.763581548)),
            config);
            
        RamseteCommand driveTarmacToLeftBall = new RamseteCommand(tarmacToLeftBall,
            m_drive::getPose2d,
            new RamseteController(1.05, 0.14),
            new SimpleMotorFeedforward(DriveConstants.kramseteS, DriveConstants.kramseteV, DriveConstants.kramseteA),
            m_drive.m_kinematics,
            m_drive::getCurrentSpeeds,
            new PIDController(DriveConstants.kLeftP, DriveConstants.kLeftI, DriveConstants.kLeftD),
            new PIDController(DriveConstants.kRightP, DriveConstants.kRightI, DriveConstants.kRightD),
            m_drive::setVoltage, m_drive);

        RamseteCommand driveLeftBallToMidBall = new RamseteCommand(leftBallToMidBall,
            m_drive::getPose2d,
            new RamseteController(1.05, 0.14),
            new SimpleMotorFeedforward(DriveConstants.kramseteS, DriveConstants.kramseteV, DriveConstants.kramseteA),
            m_drive.m_kinematics,
            m_drive::getCurrentSpeeds,
            new PIDController(DriveConstants.kLeftP, DriveConstants.kLeftI, DriveConstants.kLeftD),
            new PIDController(DriveConstants.kRightP, DriveConstants.kRightI, DriveConstants.kRightD),
            m_drive::setVoltage, m_drive);

        RamseteCommand driveMidBallToTerminal = new RamseteCommand(midBallToTerminal,
            m_drive::getPose2d,
            new RamseteController(1.05, 0.14),
            new SimpleMotorFeedforward(DriveConstants.kramseteS, DriveConstants.kramseteV, DriveConstants.kramseteA),
            m_drive.m_kinematics,
            m_drive::getCurrentSpeeds,
            new PIDController(DriveConstants.kLeftP, DriveConstants.kLeftI, DriveConstants.kLeftD),
            new PIDController(DriveConstants.kRightP, DriveConstants.kRightI, DriveConstants.kRightD),
            m_drive::setVoltage, m_drive);

        addCommands(
            // Without shooting/intake
            // driveTarmacToMidBall,
            // new WaitCommand(3),
            // driveMidBallToTerminal

            // With shooting/intake
            new BasicAutoNoMove(), // Robot must face away from basket
            new ParallelRaceGroup(new IntakeBalls(), driveTarmacToLeftBall),
            new MoveToAngleLime(VisionConstants.kRotP_lime, VisionConstants.kVecP_lime),
            new ParallelCommandGroup(new InstantCommand(() -> Robot.hood.setAngle(45))),
            new ParallelRaceGroup(new ShootBall(), new WaitCommand(5)),

            new ParallelRaceGroup(new IntakeBalls(), driveLeftBallToMidBall),
            new ParallelRaceGroup(new OpenLoopDrive(Robot.drive, -0.2), new WaitCommand(1)),
            new MoveToAngleLime(VisionConstants.kRotP_lime, VisionConstants.kVecP_lime),
            new ParallelCommandGroup(new InstantCommand(() -> Robot.hood.setAngle(45))),
            new ParallelRaceGroup(new ShootBall(), new WaitCommand(5)),

            new ParallelRaceGroup(new IntakeBalls(), driveMidBallToTerminal),
            new ParallelRaceGroup(new OpenLoopDrive(Robot.drive, -0.2), new WaitCommand(1)),
            new MoveToAngleLime(VisionConstants.kRotP_lime, VisionConstants.kVecP_lime),
            new ParallelCommandGroup(new InstantCommand(() -> Robot.hood.setAngle(45))),
            new ParallelRaceGroup(new ShootBall(), new WaitCommand(5))
        );

    }

}
