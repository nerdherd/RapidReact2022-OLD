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

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.IntakeBalls;
import frc.robot.constants.DriveConstants;
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

        Trajectory tarmacToTerminal = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 91.37500, new Rotation2d(0.25)),
            List.of(new Translation2d(29.8244158, 150.06500),
                    new Translation2d(Math.cos(DriveConstants.kTarmacToBallOne) * 16.867, Math.sin(DriveConstants.kTarmacToBallOne) * 16.867)
            ),
            new Pose2d(275.24, 132.93, new Rotation2d(0.449923)),
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
            new BasicAutoNoMove(),
            new ParallelRaceGroup(new IntakeBalls(), driveTarmacToTerminal),
            new WaitCommand(3)
        );

    }

}
