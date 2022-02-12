<<<<<<< HEAD:src/main/java/frc/robot/commands/climber/ClimberLift.java
// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot.commands.climber;

// import com.nerdherd.lib.motor.commands.SetMotorPower;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Robot;
// import frc.robot.constants.ClimberConstants;
// import frc.robot.subsystems.Climber;

// public class ClimberLift extends CommandBase {
//     /**
//      * Creates a new ClimberLift.
//      */
//     public ClimberLift() {
//         addRequirements(Robot.climber);
//     }

//     // Called when the command is initially scheduled.
//     @Override
//     public void initialize() {
//     }

//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {
//         // if(Robot.climber.mainFalcon.getPosition() > ClimberConstants.kClimbGoodPos){
//         // Robot.climber.setPower(ClimberConstants.kClimberDesiredHoldPow);
//         // }else{
//         // Robot.climber.setPower(ClimberConstants.kClimberDesiredLiftPow);
//         // }
//         if (Robot.climber.mainFalcon.getPosition() < ClimberConstants.kClimbGoodPos
//                 && Robot.climber.followerFalcon.getPosition() < ClimberConstants.kClimbGoodPos) {
//             Robot.climber.setPower(ClimberConstants.kClimberDesiredHoldPow);
//         } else {
//             Robot.climber.setPower(ClimberConstants.kClimberDesiredLiftPow);
//         }
//     }

//     // Called once the command ends or is interrupted.
//     @Override
//     public void end(boolean interrupted) {
//         Robot.climber.setPower(0);
//     }

//     // Returns true when the command should end.
//     @Override
//     public boolean isFinished() {
//         // return Robot.climber.getHeight() < ClimberConstants.kClimbGoodPos;
//         return false;

//     }
// }
=======
package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.constants.ClimberConstants;

public class ArmDetectCurrent extends CommandBase{
    
    public ArmDetectCurrent() {
        addRequirements(Robot.arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Robot.arm.getCurrent() > ClimberConstants.kArmDesiredHoldPower + ClimberConstants.kArmHitCurrentChange) {
            return true;
        } else {
            return false;
        }
    }

}
>>>>>>> master:src/main/java/frc/robot/commands/climber/ArmDetectCurrent.java
