// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

<<<<<<< HEAD
// package frc.robot.commands.climber;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
// public class ClimberClimb extends SequentialCommandGroup {
//     /**
//      * Creates a new ClimberClimb.
//      */
//     public ClimberClimb() {
//         super();
//         new ClimberReady();
//         new ClimberLift();
//         // Add your commands in the super() call, e.g.
//         // super(new FooCommand(), new BarCommand());
//     }
// }
=======
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

public class ClimberClimb extends SequentialCommandGroup{
    
    // Robot.arm.detectCurrent() should have isFinished return true if current goes out of range

    public ClimberClimb() {
        addCommands(
            new InstantCommand(Robot.arm.setAngle(ClimberConstants.kArmRotBack)),
            new InstantCommand(Robot.armElevator.setHeight(ClimberConstants.kArmTravelUp)),
            new ParallelRaceGroup(Robot.arm.setAngle(ClimberConstants.kArmRotForwards), new ArmDetectCurrent()),
            new InstantCommand(Robot.armElevator.setHeight(ClimberConstants.kArmTravelDown))
        );
    }

}
>>>>>>> master
