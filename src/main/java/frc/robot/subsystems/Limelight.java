package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase {
  /**
   * Creates a new Limelight.
   */
  
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry ts;
  NetworkTableEntry tlong;
  NetworkTableEntry pipeline;
  NetworkTableEntry ledMode;
  NetworkTableEntry camMode;

  public Limelight() {
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    ts = table.getEntry("ts");
    tlong = table.getEntry("tlong");
    pipeline = table.getEntry("pipeline");
    camMode = table.getEntry("camMode");
    ledMode = table.getEntry("ledMode");

    pipeline.setValue(0);
    camMode.setValue(0);

  }

  public void togglePipeline(){
    pipeline.setValue(1.0);
    

  }

  public double getTargetWidth(){
    double w = tlong.getDouble(0.0);
    return w;
  }

  public double getXOffsetFromTarget() {
    // double x = tx.getDouble(0.0) + 1;
    double x = tx.getDouble(0.0);
    return x + 1;
  }

  public double getYOffsetFromTarget() {
    double y = ty.getDouble(0.0);
    return y;
  }

  public double getTargetArea() {
    double area = ta.getDouble(0.0);
    return area;
  }

  public double getTargetAngle(){
    double skew = ts.getDouble(0.0);
    return skew;
  }

  public double getDistance(){
    // distance is head-on
    double dist = (VisionConstants.kTargetHeight-VisionConstants.kCameraMountHeight) / Math.tan(getYOffsetFromTarget());
    return dist;
}
  public double getDistanceWidth(){
    // distance is at an angle
    double distanceHyp = (VisionConstants.kTargetWidth * VisionConstants.kXFocalLength_lime)/getTargetWidth(); // D = W*F/P; convert to in
    double theta = Math.asin((VisionConstants.kTargetHeight-VisionConstants.kCameraMountHeight)/distanceHyp); // angle of elevation to target
    double fullDist = distanceHyp*Math.cos(theta); // horizontal distance
    // SmartDashboard.putNumber("distanceHyp", distanceHyp);
    // SmartDashboard.putNumber("theta", theta);
    SmartDashboard.putNumber("Limelight Dist No Offset", fullDist);  
    return (fullDist - VisionConstants.kCameraBumperOffset); //PLEASE SUBTRACT BUMPER LATER
    
  }

  public void blink() {
    ledMode.setNumber(2);
  }

  public void setOff(){
    ledMode.setNumber(1);
  }

  public void setOn() {
    ledMode.setNumber(0);
  }

  public void reportToSmartDashboard() {
    SmartDashboard.putNumber("Limelight X", getXOffsetFromTarget());
    SmartDashboard.putNumber("Limelight Y", getYOffsetFromTarget());
    SmartDashboard.putNumber("Limelight Area", getTargetArea());
    // SmartDashboard.putNumber("Limelight Distance Vert.", getDistance());
    SmartDashboard.putNumber("Limelight Distance Offset.", getDistanceWidth());

    SmartDashboard.putNumber("Limelight Target Width", getTargetWidth());
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
