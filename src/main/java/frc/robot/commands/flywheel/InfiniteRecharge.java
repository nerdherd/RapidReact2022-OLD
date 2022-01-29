/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.flywheel;

import com.nerdherd.lib.motor.commands.SetMotorVelocity;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.constants.ShooterConstants;

public class InfiniteRecharge extends SetMotorVelocity {
  /**
   * Creates a new InfiniteRecharge.
   */
  public InfiniteRecharge() {
    super(Robot.shooter, 16200);
    // Use addRequirements() here to declare subsystem dependencies.
  }

}