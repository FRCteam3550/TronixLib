package org.usfirst.frc3550.Julius2018.subsystems;

import org.opencv.core.Mat;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



/**
 *
 */
public class Cameras extends Subsystem {
	UsbCamera cam1;
	UsbCamera cam2;
	int currentCamera = 0;
	int cameraCount = 2;
	Mat img = new Mat();
	public boolean isVisionMode = false;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    
	public void initDefaultCommand() {
		cam1 = CameraServer.getInstance().startAutomaticCapture(0);
		cam1.setResolution(640, 480);
		cam2 = CameraServer.getInstance().startAutomaticCapture(1);
		cam2.setResolution(640, 480);
		cam1.setFPS(30);
		cam2.setFPS(5);
		//SmartDashboard.putBoolean("GearCamFocus", false);
		//SmartDashboard.putBoolean("MainCamFocus", false);
    }
    
    public void visionMode() {
    	cam1.setExposureManual(2);
    	cam1.setExposureHoldCurrent();
    	cam1.setFPS(30);
		cam2.setFPS(5);//
		isVisionMode = true;
    }
    
    public void visionModeLite() {
    	cam1.setExposureManual(2);
    	cam1.setExposureHoldCurrent();
    	isVisionMode = true;
    }
    
    public void drivingMode() {
    	cam1.setExposureManual(40);
    	cam1.setExposureHoldCurrent();
    	cam1.setFPS(30);//default value 20
		cam2.setFPS(30); //default value 20
		isVisionMode = false;
    }
}

