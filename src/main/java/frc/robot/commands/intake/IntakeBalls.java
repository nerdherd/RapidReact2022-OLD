/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

// import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Indexer.IndexerState;

public class IntakeBalls extends CommandBase {

  private double m_startTime;
  private int state, counter;
  /**
   * Creates a new Intake.
   */
  public IntakeBalls() {
    state = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.intake, Robot.intakeRoll);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Robot.hopper.setPower(-0.33, -0.167);
    Robot.hopper.setPower(-0.33, 0.167);
    m_startTime = Timer.getFPGATimestamp();
    if (Robot.indexer.indexerState == IndexerState.EMPTY) {
      Robot.indexer.indexerState = IndexerState.WAITING;
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.intakeRoll.setPower(0.95);
    Robot.intake.setForwards();
    Robot.shooter.setPower(0.0);

    if (Robot.indexer.indexerBallDetected()){
      Robot.indexer.setPower(0.0);
      Robot.hopper.setPowerWithoutTop(0.0, 0.0);
    }else{
      Robot.indexer.setPower(0.16);
    }

    if (Timer.getFPGATimestamp() - m_startTime > 1.5) {
      Robot.hopper.setTopHopperPower(0.67); 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
