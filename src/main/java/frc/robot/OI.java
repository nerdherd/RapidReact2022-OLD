package frc.robot;

import com.nerdherd.lib.motor.commands.SetMotorPower;
import com.nerdherd.lib.oi.DefaultOI;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auto.BasicAuto;
import frc.robot.commands.auto.TarmacToTerminalThree;
import frc.robot.commands.climber.ClimberClimb;
import frc.robot.commands.climber.ClimberReady;
import frc.robot.commands.climber.ExtendElevator;
import frc.robot.commands.climber.RotateArmFor;
import frc.robot.commands.climber.RotateArmBack;
import frc.robot.commands.hood.SetAngle;
import frc.robot.commands.intake.IntakeBalls;
import frc.robot.commands.intake.Stow;
import frc.robot.commands.shooting.ShootBall;
import frc.robot.commands.vision.MoveToAngleLime;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.VisionConstants;

public class OI extends DefaultOI {
    public JoystickButton intake_1, startShooting_2, startShootingOld_3, trenchShot_7, autolineShot_9, stow_10,
            wallShot_11, autoDistance_12, hoodAngle_5, turnToAngle_1L, turnToAngle_1R, resetEncoders_5R,
            resetEncoders_5L, outtake_6, shiftHigh_6L, shiftLow_6R, ploughIntake_2, togglePipeline_4, rendezvousShot_8,
            climbReady_3L, climbLift_4L, outtakeBrushes_8;

    public OI() {
        super();
        resetEncoders_5L = new JoystickButton(super.driveJoyLeft, 5);
        resetEncoders_5R = new JoystickButton(super.driveJoyRight, 5);
        intake_1 = new JoystickButton(super.operatorJoy, 1);
        ploughIntake_2 = new JoystickButton(super.driveJoyLeft, 2);
        startShooting_2 = new JoystickButton(super.operatorJoy, 2);
        stow_10 = new JoystickButton(super.operatorJoy, 10);
        hoodAngle_5 = new JoystickButton(super.operatorJoy, 5);

        resetEncoders_5L.whenPressed(Robot.hoodReset);
        resetEncoders_5R.whenPressed(Robot.hoodReset);
        intake_1.whenPressed(new IntakeBalls());
        ploughIntake_2.whenPressed(new SetMotorPower(Robot.intakeRoll, -0.75));
        startShooting_2.whileHeld(new ShootBall());
        hoodAngle_5.whenPressed(new SetAngle());
        stow_10.whenPressed(new Stow());

        // CLIMBER
        SmartDashboard.putData("Climber Ready", new ClimberReady());
        SmartDashboard.putData("Climber Climb", new ClimberClimb());

        // AUTOS
        SmartDashboard.putData("MoveToAngle", new MoveToAngleLime(VisionConstants.kRotP_lime, VisionConstants.kVecP_lime));
        SmartDashboard.putData("Basic Auto", new BasicAuto());
        SmartDashboard.putData("Tarmac to Terminal Five", new TarmacToTerminalThree(Robot.drive));

        SmartDashboard.putData("Extend Elevator", new ExtendElevator());
        SmartDashboard.putData("Rotate Arm Forwards", new RotateArmFor());
        SmartDashboard.putData("Rotate Arm Backwards", new RotateArmBack());
    }
}
