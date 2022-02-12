// package frc.robot.commands.climber;

// import com.nerdherd.lib.motor.commands.SetMotorPower;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Robot;
// import frc.robot.constants.ClimberConstants;
// import frc.robot.subsystems.Climber;

// public class ClimberReady extends CommandBase {
//     /**
//      * Creates a new ClimberReady.
//      */
//     public ClimberReady() {
//         addRequirements(Robot.climber);
//         // Use addRequirements() here to declare subsystem dependencies.
//     }

//     // Called when the command is initially scheduled.
//     @Override
//     public void initialize() {
//         super.initialize();
//     }

//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {
//         Robot.climber.setPower(1);
//     }

//     // Called once the command ends or is interrupted.
//     @Override
//     public void end(boolean interrupted) {
//         Robot.climber.setPower(ClimberConstants.kClimberDesiredUpPow);

//     }

//     // Returns true when the command should end.
//     @Override
//     public boolean isFinished() {
//         return Robot.climber.mainFalcon.getPosition() > ClimberConstants.kHardStopPos
//                 && Robot.arm.followerFalcon.getPosition() > ClimberConstants.kHardStopPos;
//     }
// }