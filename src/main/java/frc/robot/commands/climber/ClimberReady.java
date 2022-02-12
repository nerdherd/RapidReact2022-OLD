/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/* In order to run this command, position robot so it is relatively close to 
bottom rung facing the hangar. The robot will drive forward until a curret 
change in arm is detected. */

package frc.robot.commands.climber;

import frc.robot.Robot;
import frc.robot.constants.ClimberConstants;
import frc.robot.commands.climber.ArmDetectCurrent;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ClimberReady extends SequentialCommandGroup{
    
    // Robot.arm.detectCurrent() should have isFinished return true if current goes out of range
    
    public ClimberReady() {
        addCommands(
            new InstantCommand(() -> Robot.elevator.setHeight(ClimberConstants.kElevatorReadyPosition)), // Assumes arm raises when elevator raises

            new ParallelRaceGroup(new InstantCommand(() -> Robot.drive.setPower(ClimberConstants.kApproachSpeed,
                ClimberConstants.kApproachSpeed)), new ArmDetectCurrent()), // Approach until arm collision

            new InstantCommand(() -> Robot.elevator.setHeight(ClimberConstants.kElevatorLiftPosition)) // Lift robot up
        );
    }

}
