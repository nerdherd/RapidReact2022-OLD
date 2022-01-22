/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import com.nerdherd.lib.drivetrain.characterization.OpenLoopDrive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.shooting.ShootBall;
import frc.robot.constants.VisionConstants;

/**
 * Add your docs here.
 */
public class BasicAutoNoMove extends SequentialCommandGroup {

    public BasicAutoNoMove() {
        addCommands(
            new InstantCommand(() -> Robot.hood.setStoredAngle(), Robot.hood), 
            // new ParallelRaceGroup(new WaitCommand(1), new TurnToAngleLime(VisionConstants.kRotP_lime)),
            new ParallelRaceGroup(new ShootBall(), new WaitCommand(2.75))
            // new ParallelRaceGroup(new OpenLoopDrive(Robot.drive, -0.2), new WaitCommand(1))
        );
    }

}