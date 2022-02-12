package frc.robot.subsystems;

import com.nerdherd.lib.motor.single.SingleMotorMechanism;
import com.nerdherd.lib.motor.motorcontrollers.NerdyTalon;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

import frc.robot.RobotMap;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.TurretConstants;

public class Turret extends SingleMotorMechanism{
    public Turret(){
        super(RobotMap.kTurretID, "Turret", false, false);
        super.configPIDF(VisionConstants.kRotP_lime, 0, 0, VisionConstants.kRotF_lime);
        super.configCurrentLimit(TurretConstants.kTurretCurrentPeak, TurretConstants.kTurretCurrentContinuous);
    }
}
