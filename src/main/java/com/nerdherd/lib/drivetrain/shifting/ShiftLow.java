/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.nerdherd.lib.drivetrain.shifting;

import com.nerdherd.lib.drivetrain.experimental.TwoSpeedDrivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class ShiftLow extends CommandBase {

  TwoSpeedDrivetrain m_drive;

  public ShiftLow(TwoSpeedDrivetrain drive) {
    m_drive = drive;
    // addRequirements(m_drive);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    m_drive.shiftLow();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  
}
