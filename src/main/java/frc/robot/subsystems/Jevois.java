package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

import frc.robot.Robot;
import frc.robot.subsystems.Jevois;
import frc.robot.constants.VisionConstants;


import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.hal.util.UncleanStatusException;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Jevois extends SubsystemBase implements Runnable {
  private SerialPort m_cam;
	private UsbCamera m_visionCam;
	private Thread m_stream;
	private boolean m_send;
	public double m_offset;
	public double m_distinguish = 0.0;
	private double m_threshold = 25.4;

	private boolean writeException = false;
	String[] parts;
	private String sendValue;
	private String m_filePath1 = "/media/sda1/logs/";
	private String m_filePath2 = "/home/lvuser/logs/";
	private File m_file;
	public FileWriter m_writer;
	private double m_logStartTime = 0;

	// Jevois Serial Output Data
	private double m_contourNum, m_area, m_centerX, m_centerY, m_distance, m_theta, m_thetaOld, m_distanceOld;

	public Jevois(int baud, SerialPort.Port port) {
		m_send = false;
		sendValue = "None";
		try {
			m_cam = new SerialPort(baud, port);
			m_stream = new Thread(this);
			m_stream.start();
		} catch (UncleanStatusException e) {
			e.printStackTrace();
		}
	}

	public void startCameraStream() {
		try {
			m_visionCam = CameraServer.startAutomaticCapture();
			m_visionCam.setVideoMode(PixelFormat.kMJPEG, 320, 240, 30);
		} catch (Exception e) {
		}
	}

	public void run() {
		while (m_stream.isAlive()) {
			Timer.delay(0.001);
			if (m_send) {
				m_cam.writeString(sendValue);
				m_send = false;
			}
			if (m_cam.getBytesReceived() > 0) {
				String read = m_cam.readString();
				if (read.charAt(0) == '/') {
					parts = dataParse(read);
					m_contourNum = Integer.parseInt(getData(1));
        //   m_distance = Double.parseDouble(getData(2));
        //   m_theta = Double.parseDouble(getData(3));
		  m_distanceOld = Double.parseDouble(getData(2));
		//   m_thetaOld = Double.parseDouble(getData(3));
          m_centerX = Double.parseDouble(getData(3));
          m_centerY = Double.parseDouble(getData(4));
				} else {
					System.out.println(read);
				}
			}
		}
	}

	// use if jevois is centered on robot
	public double getSimpleAngleToTurn() {
		return xPixelToDegree(getTargetX());
	}

	// use if jevois is horizontally offset from center
	public double getAngleToTurn() {
		double radians = (Math.PI / 180) * (xPixelToDegree(getTargetX()) 
						+ VisionConstants.kCameraHorizontalMountAngle);
		double horizontalAngle = Math.PI / 2 - radians;
		double distance = getOldDistance();
		double f = Math.sqrt(distance * distance + Math.pow(VisionConstants.kCameraHorizontalOffset, 2)
				- 2 * distance * VisionConstants.kCameraHorizontalOffset * Math.cos(horizontalAngle));
		double c = Math.asin(VisionConstants.kCameraHorizontalOffset * Math.sin(horizontalAngle) / f);
		double b = Math.PI - horizontalAngle - c;
		double calculatedAngle = (180 / Math.PI) * (Math.PI / 2 - b);
		if (getTargetX() == 0) {
			return 0;
		} else {
			return calculatedAngle;
		}
	}

	private double xPixelToDegree(double pixel) {
		double radian = Math.signum(pixel) * Math.atan(Math.abs(pixel / VisionConstants.kXFocalLength));
		double degree = 180 / Math.PI * radian;
		return degree;
	}

	public double getDistance() {
		return m_distance;
  }
  public double getOldDistance(){
    return m_distanceOld;
  }

	public double getDistanceFeet() {
		return m_distance / 12;
	}

	// use if overshooting the turn
	public double getOffset() {
		double angularError = getSimpleAngleToTurn();
		if (-m_threshold < angularError && angularError < m_threshold) {
			m_offset = 0.0;
		} else if (m_threshold > 0) {
			m_offset = angularError - m_threshold;
		} else if (m_threshold < 0) {
			m_offset = angularError + m_threshold;
		}
		return m_offset;
	}

	
	public double getContourNum() {
		return m_contourNum;
	}

	public double getTargetX() {
		return m_centerX;
	}

	public double getTargetY() {
		return m_centerY;
	}

	public double getTargetArea() {
		return m_area;
  }
  
  public double getAngle(){
    return m_theta;
  }


	public void end() {
		m_stream.interrupt();
	}

	private void sendCommand(String value) {
		sendValue = value + "\n";
		m_send = true;
		Timer.delay(0.1);
	}

	private String[] dataParse(String input) {
		return input.split("/");
	}

	private String getData(int data) {
		return parts[data];
	}

	public void ping() {
		sendCommand("ping");
	}

	public void enableStream() {
		sendCommand("streamon");
	}

	public void disableStream() {
		sendCommand("streamoff");
	}

	public void reportToSmartDashboard() {
		SmartDashboard.putNumber("Total contours", getContourNum());
		//SmartDashboard.putNumber("Area", getTargetArea());
		SmartDashboard.putNumber("Y coord", getTargetY());
		SmartDashboard.putNumber("X coord", getTargetX());
    SmartDashboard.putNumber("Old Angle to Turn", getAngleToTurn());
    SmartDashboard.putNumber("New Theta", getAngle());
		SmartDashboard.putNumber("Distance", getOldDistance());
		SmartDashboard.putBoolean("Contour detected", getContourNum() > 0);
		SmartDashboard.putBoolean("Is Locked on", Math.abs(getAngleToTurn()) <= VisionConstants.kDriveRotationDeadband);

	//	SmartDashboard.putNumber("Exposure", getExp());

	}


	public void startLog() {
		// Check to see if flash drive is mounted.
		File logFolder1 = new File(m_filePath1);
		File logFolder2 = new File(m_filePath2);
		Path filePrefix = Paths.get("");
		if (logFolder1.exists() && logFolder1.isDirectory()) {
			filePrefix = Paths.get(logFolder1.toString(),
					Robot.kDate + Robot.ds.getMatchType().toString() + Robot.ds.getMatchNumber() + "Jevois");
		} else if (logFolder2.exists() && logFolder2.isDirectory()) {
			filePrefix = Paths.get(logFolder2.toString(),
					Robot.kDate + Robot.ds.getMatchType().toString() + Robot.ds.getMatchNumber() + "Jevois");
		} else {
			writeException = true;
		}

		if (!writeException) {
			int counter = 0;
			while (counter <= 99) {
				m_file = new File(filePrefix.toString() + String.format("%02d", counter) + ".csv");
				if (m_file.exists()) {
					counter++;
				} else {
					break;
				}
				if (counter == 99) {
					System.out.println("file creation counter at 99!");
				}
			}
			try {
				m_writer = new FileWriter(m_file);
				m_writer.append("Time,Contours,Area,TargetX,TargetY,Distance,Angle\n");
				m_logStartTime = Timer.getFPGATimestamp();
			} catch (IOException e) {
				e.printStackTrace();
				writeException = true;
			}
		}
	}

	public void stopLog() {
		try {
			if (null != m_writer)
				m_writer.close();
		} catch (IOException e) {
			e.printStackTrace();
			writeException = true;
		}
	}

	public void logToCSV() {
		if (!writeException) {
			try {
				double timestamp = Timer.getFPGATimestamp() - m_logStartTime;
				m_writer.append(String.valueOf(timestamp) + ","
						+ String.valueOf(getContourNum()) + "," + String.valueOf(getTargetArea()) + ","
						+ String.valueOf(getTargetX()) + "," + String.valueOf(getTargetY()) +"," 
            + String.valueOf(getDistance()) + "," + String.valueOf(getAngleToTurn()) + ","
            + String.valueOf(getAngle()) + "\n");
				m_writer.flush();
			} catch (IOException e) {
				e.printStackTrace();
				writeException = true;
			}
		}
	}


}