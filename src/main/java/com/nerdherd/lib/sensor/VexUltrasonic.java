package com.nerdherd.lib.sensor;

import com.nerdherd.lib.logging.Loggable;
import com.nerdherd.lib.logging.NerdyBadlog;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class VexUltrasonic extends Ultrasonic implements Loggable {

  public String name;

  public VexUltrasonic(String name, int ping, int echo){
    super(ping, echo);
    super.setAutomaticMode(true);

    this.name = name;
  }

  public double getInches(){
    //accurate from 2 to 40 (could not accuratly test > 40)
    return super.getRangeInches();
  } 

  // public double getInchesWithTolerance(){
    // return super.getRangeInches() - 8.25;  
  // }

  public void reportToSmartDashboard(){
    SmartDashboard.putNumber(name + " inches", getInches());
  }

  @Override
  public void initLoggingData() {
    NerdyBadlog.createTopic(name + "/Inches", () -> this.getInches());
  }

}
