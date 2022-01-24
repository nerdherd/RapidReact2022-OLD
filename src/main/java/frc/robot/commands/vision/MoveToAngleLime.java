package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;
import frc.robot.constants.VisionConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// 2022 VISION TESTING ON THOMAS, WILL NOT WORK ON 2022 WPILIB WITHOUT AN IMPORT

public class MoveToAngleLime extends CommandBase {
    private double m_rotP;
    private double m_vecP;

    public MoveToAngleLime(double k_rotP, double k_vecP) {
        addRequirements(Robot.drive);
        addRequirements(Robot.limelight);
        m_rotP = k_rotP;
        m_vecP = k_vecP;
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Current Command", "MoveToAngle");
        Robot.limelight.setOn();
    }

    @Override
    public void execute() {
        double getAngularTargetError = Robot.limelight.getXOffsetFromTarget();
        double getVectorTargetError = Robot.limelight.getDistance();
        double robotAngle = (360 - Robot.drive.getRawYaw()) % 360;
        double rot_power = -m_rotP * getAngularTargetError;
        double vec_power = m_vecP * getVectorTargetError;

        if (!(Math.abs(getAngularTargetError) < VisionConstants.kDriveRotationDeadband)) {
            Robot.drive.setPowerFeedforward(-rot_power, rot_power);
        } else if (!(Math.abs(getAngularTargetError) > VisionConstants.kDriveRotationDeadband)) {
            Robot.drive.setPowerFeedforward(rot_power, -rot_power);
        } else {
            Robot.drive.setPowerFeedforward(0.0, 0.0);
        }

        if (!(Math.abs(getVectorTargetError) < VisionConstants.kShortShot)) {
            Robot.drive.setPowerFeedforward(vec_power, vec_power);
        } else if (!(Math.abs(getVectorTargetError) > VisionConstants.kShortShot)) {
            Robot.drive.setPowerFeedforward(-vec_power, -vec_power);
        } else {
            Robot.drive.setPowerFeedforward(0.0, 0.0);
        }

        SmartDashboard.putNumber("Left Rotational Power", rot_power);
        SmartDashboard.putNumber("Right Rotational Power", -rot_power);
        SmartDashboard.putNumber("Left Vector Power", vec_power);
        SmartDashboard.putNumber("Right Vector Power", -vec_power);
        SmartDashboard.putNumber("Robot Angle", robotAngle);
    }
}