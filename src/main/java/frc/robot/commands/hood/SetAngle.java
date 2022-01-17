/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hood;

import com.nerdherd.lib.motor.commands.mechanisms.SetArmAngleMotionMagic;
import frc.robot.Robot;

public class SetAngle extends SetArmAngleMotionMagic {
  /**
   * Creates a new SetAngle.
   */
  public SetAngle() {
    super(Robot.hood, Robot.hood.storedAngle);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  @Override
  public void execute() {
    // TODO Auto-generated method stub
    Robot.hood.setAngleMotionMagic(Robot.hood.storedAngle);
  }

}
