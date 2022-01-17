package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.constants.ShooterConstants;

public class ShootBall extends CommandBase {

  int isSafeToShoot = 0;

  /**
   * Creates a new ShootBall.
   */
  public ShootBall() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.indexer, Robot.hopper, Robot.hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isSafeToShoot = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(Robot.shooter.getVelocity() - Robot.shooter.getDesiredVel())
          < ShooterConstants.kVelPercentTolerance * Robot.shooter.getDesiredVel()){
      isSafeToShoot++;
    } else {
      isSafeToShoot = 0;
    }
    if (isSafeToShoot > 5) {
      Robot.indexer.setPower(0.6);
      Robot.hopper.setPower(0.4, 0.8);
    }else{
      Robot.indexer.setPower(0.0);
      Robot.hopper.setPower(0.0, 0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.indexer.setPower(0);
    Robot.hopper.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
