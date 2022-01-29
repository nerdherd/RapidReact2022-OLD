package frc.robot.commands.auto;

import java.time.Instant;

import com.nerdherd.lib.drivetrain.auto.DriveTime;
import com.nerdherd.lib.drivetrain.experimental.Drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.flywheel.StartFlywheel;
import frc.robot.commands.intake.Stow;
import frc.robot.commands.shooting.ShootBall;
import frc.robot.commands.vision.MoveToAngleLime;
import frc.robot.constants.VisionConstants;

public class TestFlywheel extends SequentialCommandGroup {
    
    private Drivetrain m_drive;

    public TestFlywheel(Drivetrain drive) {

        addCommands(
            new ShootBall()
        );

    }

}
