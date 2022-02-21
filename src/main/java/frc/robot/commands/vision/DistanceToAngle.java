package frc.robot.commands.vision;

import com.nerdherd.lib.motor.commands.mechanisms.SetArmAngleMotionMagic;

import frc.robot.Robot;
import frc.robot.constants.ShooterConstants;

/**
 * Add your docs here.
 */
public class DistanceToAngle extends SetArmAngleMotionMagic {
    public DistanceToAngle(){
        super(Robot.hood, Robot.hood.storedAngle);
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        Robot.hood.setAngleMotionMagic(Robot.hood.distToAngle(Robot.limelight.getDistanceWidth()));
        Robot.shooter.setVelocity(ShooterConstants.kAutoAngleCloseVelocity);
        
    }
}
