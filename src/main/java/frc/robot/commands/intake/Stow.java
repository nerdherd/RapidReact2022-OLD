/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.constants.HoodConstants;

public class Stow extends CommandBase {
  /**
   * Creates a new Stow.
   */
  public Stow() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.intake, Robot.intakeRoll, Robot.hopper, Robot.hood, Robot.indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.intake.setReverse();
    Robot.intakeRoll.setPower(0);
    Robot.hopper.setPowerWithoutTop(0, 0);
    Robot.hopper.setTopHopperPower(0.4167);
    Robot.hood.setAngle(HoodConstants.kStowAngle);
    Robot.indexer.setPower(0);
    Robot.shooter.setVelocity(0);
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
