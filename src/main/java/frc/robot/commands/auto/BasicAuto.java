/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.commands.auto;

import com.nerdherd.lib.drivetrain.auto.DriveTime;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.intake.Stow;
import frc.robot.commands.shooting.ShootBall;

public class BasicAuto extends SequentialCommandGroup {
    
    public BasicAuto() {
        addCommands(
            new ShootBall(),
            new InstantCommand(() -> Robot.hood.setStoredAngle(), Robot.hood),
            new DriveTime(Robot.drive, -0.2, 1),
            new Stow(),
            new WaitCommand(1),
            new InstantCommand(() -> Robot.hood.resetEncoder())
        );
    }

}
