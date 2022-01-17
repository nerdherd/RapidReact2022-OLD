package frc.robot;

import com.nerdherd.lib.oi.DefaultOI;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class OI extends DefaultOI {
    public JoystickButton intake_1, startShooting_2, startShootingOld_3, trenchShot_7, autolineShot_9, stow_10,
            wallShot_11, autoDistance_12, hoodAngle_5, turnToAngle_1L, turnToAngle_1R, resetEncoders_5R,
            resetEncoders_5L, outtake_6, shiftHigh_6L, shiftLow_6R, ploughIntake_2, togglePipeline_4, rendezvousShot_8,
            climbReady_3L, climbLift_4L, outtakeBrushes_8;

    public OI() {
        super();
    }
}
