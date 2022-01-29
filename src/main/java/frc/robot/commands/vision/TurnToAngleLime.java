package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;
import frc.robot.constants.VisionConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnToAngleLime extends CommandBase {

    private double m_rotP;

    public TurnToAngleLime(double kRotP) {
        addRequirements(Robot.drive);
        addRequirements(Robot.limelight);
        m_rotP = kRotP;
    }

    @Override

    public void initialize() {
        // Robot.jevois.enableStream();
        SmartDashboard.putString("Current Command", "TurnToAngle");
        Robot.limelight.setOn();
    }

    @Override
    public void execute() {
        double getAngularTargetError = Robot.limelight.getXOffsetFromTarget();
        double robotAngle = (360 - Robot.drive.getRawYaw()) % 360;
        double power = -m_rotP * getAngularTargetError;
        if (!(Math.abs(getAngularTargetError) < VisionConstants.kDriveRotationDeadband)) {
            Robot.drive.setPowerFeedforward(-power, power);
        } else {
            Robot.drive.setPowerFeedforward(0.0, 0.0);
        }

        SmartDashboard.putNumber("Left Power", power);
        SmartDashboard.putNumber("Right Power", -power);
        SmartDashboard.putNumber("Robot Angle", robotAngle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.drive.setPowerZero();
    }

}