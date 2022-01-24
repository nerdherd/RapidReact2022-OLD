/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.commands.auto;

import java.util.List;
import java.lang.Math;

import com.nerdherd.lib.drivetrain.experimental.Drivetrain;

import frc.robot.Robot;
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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TarmacToTerminalThree extends SequentialCommandGroup {
    
    private Drivetrain m_drive;

    public TarmacToTerminalThree(Drivetrain drive) {
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveConstants.kramseteS, DriveConstants.kramseteV, DriveConstants.kramseteA),
        m_drive.m_kinematics, 
        DriveConstants.kRamseteMaxVolts);

        var autoCentripetalAccelerationConstraint = new CentripetalAccelerationConstraint(DriveConstants.kMaxCentripetalAcceleration);
    
        TrajectoryConfig config = new TrajectoryConfig(DriveConstants.kDriveMaxVel, DriveConstants.kDriveMaxAccel);
        config.addConstraints(List.of(autoVoltageConstraint, autoCentripetalAccelerationConstraint));

        Trajectory tarmacToTerminal = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(Math.cos(DriveConstants.kTarmacToBallOneAngle) * 16.867, Math.sin(DriveConstants.kTarmacToBallOneAngle) * 16.867)),
            new Pose2d(6.991096, 3.376422, new Rotation2d(0.449923)),
            config);

        RamseteCommand driveTarmacToTerminal = new RamseteCommand(tarmacToTerminal,
            m_drive::getPose2d,
            new RamseteController(1.05, 0.14),
            new SimpleMotorFeedforward(DriveConstants.kramseteS, DriveConstants.kramseteV, DriveConstants.kramseteA),
            m_drive.m_kinematics,
            m_drive::getCurrentSpeeds,
            new PIDController(DriveConstants.kLeftP, DriveConstants.kLeftI, DriveConstants.kLeftD),
            new PIDController(DriveConstants.kRightP, DriveConstants.kRightI, DriveConstants.kRightD),
            m_drive::setVoltage, m_drive);

        addCommands(
            new ParallelRaceGroup(new IntakeBalls(), driveTarmacToTerminal),
            new ParallelCommandGroup(new InstantCommand(() -> Robot.hood.setAngle(45))),
            new ParallelRaceGroup(new ShootBall(), new WaitCommand(5)),
            new WaitCommand(3)
        );

    }

}
