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
        if (Robot.arm.getCurrent() > ClimberConstants.kArmDesiredHoldPow + ClimberConstants.kArmHitCurrentChange) {
            return true;
        } else {
            return false;
        }
    }

}
