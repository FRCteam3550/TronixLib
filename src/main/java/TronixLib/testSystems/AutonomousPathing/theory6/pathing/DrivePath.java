package org.usfirst.frc3550.Julius2018.theory6.pathing;

import java.io.BufferedOutputStream;
import java.io.FileOutputStream;
import java.io.FilterOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc3550.Julius2018.Robot;
import org.usfirst.frc3550.Julius2018.theory6.pathing.BezierCurve;
import org.usfirst.frc3550.Julius2018.theory6.pathing.Point;

import edu.wpi.first.wpilibj.command.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Command used to allow robot to travel a path generated using Bezier curves
 * 
 * @author Mahrus Kazi
 */
public class DrivePath extends Command {

	// Create a Bezier curve object
	private BezierCurve curve;

	// Variables to store parameter information
	private int counter;
	private double pathLenght;
	private double timeOut;
	private double speed;
	private boolean reverse;
	
	private ArrayList<Double> tEncoderValueslist1;
	private ArrayList<Double> tCurveLenghtValuesList1;
	
	private ArrayList<Double> tEncoderValueslist2;
	private ArrayList<Double> tCurveLenghtValuesList2;
	
	private ArrayList<Double> tCounterValueslist1;
	private ArrayList<Double> tCounterValuesList2;
	
	private ArrayList<Float> timelist;
	private float startCounter;

	/** The drive PID controller. */
	//public PIDController drivePID;

	/** The gyro PID conteroller. */
	//public PIDController gyroPID;
	
	//private  double epsilon = 1;
	//private double  angle;
	//private double output;
	
	/**
	 * Instantiates a new drive path.
	 *
	 * @param startPoint
	 *            The start point
	 * @param controlPoint1
	 *            The control point 1
	 * @param controlPoint2
	 *            The control point 2
	 * @param endPoint
	 *            The end point
	 * @param timeOut
	 *            The time out in seconds
	 * @param speed
	 *            The speed the robot will travel at (0.0 - 1.0)
	 */
	
	
	public DrivePath(Point startPoint, Point controlPoint1, Point controlPoint2, Point endPoint, double timeOut,
			double speed) {

		this(startPoint, controlPoint1, controlPoint2, endPoint, timeOut, speed, false);
		//drivePID = new PIDController(NumberConstants.pDrive, NumberConstants.iDrive, NumberConstants.dDrive);
		//gyroPID = new PIDController(NumberConstants.pGyro, NumberConstants.iGyro, NumberConstants.dGyro);
	}

	/**
	 * Instantiates a new drive path.
	 *
	 * @param startPoint
	 *            The start point
	 * @param controlPoint1
	 *            The control point 1
	 * @param controlPoint2
	 *            The control point 2
	 * @param endPoint
	 *            The end point
	 * @param timeOut
	 *            The time out in seconds
	 * @param speed
	 *            The speed the robot will travel at (0.0 - 1.0)
	 * @param reverse
	 *            True if robot will traverse path in reverse, otherwise false
	 */
	public DrivePath(Point startPoint, Point controlPoint1, Point controlPoint2, Point endPoint, double timeOut,
			double speed, boolean reverse) {

		curve = new BezierCurve(startPoint, controlPoint1, controlPoint2, endPoint);
		pathLenght = curve.findArcLength();
		//distance = 0;
		this.timeOut = timeOut;
		this.speed = speed;
		this.reverse = reverse;
		//drivePID = new PIDController(NumberConstants.pDrive, NumberConstants.iDrive, NumberConstants.dDrive);
		//gyroPID = new PIDController(NumberConstants.pGyro, NumberConstants.iGyro, NumberConstants.dGyro);
		requires(Robot.driveTrain);
	}
	 public static void recordPathingData1(List<Float> dataTime,List<Double> data, String filename1,String filename2) {

	    	try { 
	    		//DataOutputStream dos= new DataOutputStream(new BufferedOutputStream(new FileOutputStream(filename1))); 
	    		FilterOutputStream fileOutputStream1 = new FilterOutputStream(new BufferedOutputStream(new FileOutputStream(filename1)));
	    		FilterOutputStream fileOutputStream = new FilterOutputStream(new BufferedOutputStream(new FileOutputStream(filename2)));
	    		
	    		PrintStream printStream1 = new PrintStream(fileOutputStream1);
	    		PrintStream printStream = new PrintStream(fileOutputStream);
	    		
	    		for(int i=0;i<data.size();i++){ 
	        		//System.out.println("before"+data.get(i)); 
	        		printStream1.println(dataTime.get(i));
	        		printStream1.println(" ");
	        		printStream.println(data.get(i));
	        		printStream.println(" ");
	        		//dos.writeDouble(data.get(i));
	        		//System.out.println("Theory 6 DrivePath Runing"); 
	        		} 
	    		printStream1.close();
	    		printStream.close();
	    		//dos.close(); 
	    		
	    		} catch (IOException e) { 
	    			System.out.println("cant create newDatafile");
	    		e.printStackTrace(); 
	    		} 
	        System.out.println("Data saved.");
	    	
	    }
	
	// Initialize the command by reseting encoders and setting time out
	protected void initialize() {
		tEncoderValueslist1 = new ArrayList<>();
		tCurveLenghtValuesList1 = new ArrayList<>();
		tEncoderValueslist2 = new ArrayList<>();
		tCurveLenghtValuesList2 = new ArrayList<>();
		tCounterValueslist1 = new ArrayList<>();
		tCounterValuesList2 = new ArrayList<>();
		
		startCounter = (float) System.nanoTime();
		timelist =  new ArrayList<>();
		
		counter = 0;
		//angle = 0;
		//output = 0;
		setTimeout(timeOut);
		Robot.driveTrain.resetEncoders();
		//drivePID = new PIDController(NumberConstants.pDrive, NumberConstants.iDrive, NumberConstants.dDrive);
		//gyroPID = new PIDController(NumberConstants.pGyro, NumberConstants.iGyro, NumberConstants.dGyro);
	}

	// Give set distance for robot to travel, at each point change angle to
	// point towards next coordinate
	protected void execute() {
		System.out.println("Theory 6 DrivePath Runing"); 
		// output = drivePID.calcPIDDrive(distance, Robot.driveTrain.getLeftEncoderDistance(), epsilon);
		// angle = gyroPID.calcPID(curve.findAngle(counter), Robot.driveTrain.getAhrs().getYaw(), epsilon);
		timelist.add(((float)(System.nanoTime()))-startCounter);
		if (reverse) {
			if (-Robot.driveTrain.getLeftEncoderDistance() > curve.findHypotenuse(counter) && counter <= curve.size())
				counter++;
			//tCounterValueslist1.add((double) counter);
			tEncoderValueslist1.add(Robot.driveTrain.getLeftEncoderDistance());
			tCurveLenghtValuesList1.add(curve.findHypotenuse(counter));
			
			SmartDashboard.putNumber("autoEncoderDistance1:", Robot.driveTrain.getLeftEncoderDistance());
			SmartDashboard.putNumber("autoCurvePartialLenght1:", curve.findHypotenuse(counter));
		Robot.driveTrain.driveStraight(-pathLenght, speed, curve.findAngle(counter), 1); //value was 1
			//Robot.driveTrain.tankDrive(((-output + angle) * speed), ((output + angle) * speed));
			//Robot.driveTrain.m_leftMotors.set(((-output + angle) * speed)); 
//			//Robot.driveTrain.m_rightMotors.set(((output + angle) * speed)); 
		} else {
			if (Robot.driveTrain.getLeftEncoderDistance() > curve.findHypotenuse(counter) && counter < curve.size())
				counter++;
			
			//tCounterValuesList2.add((double) counter);
			tEncoderValueslist2.add(Robot.driveTrain.getLeftEncoderDistance());
			tCurveLenghtValuesList2.add(curve.findHypotenuse(counter));
			
			
			SmartDashboard.putNumber("autoEncoderDistance2:", Robot.driveTrain.getLeftEncoderDistance());
			SmartDashboard.putNumber("autoCurvePartialLenght2:", curve.findHypotenuse(counter));
//
			Robot.driveTrain.driveStraight(pathLenght, speed, curve.findAngle(counter), 1);
//			//Robot.driveTrain.tankDrive(((output + angle) * speed), ((-output + angle) * speed));
//			//Robot.driveTrain.m_leftMotors.set(((output + angle) * speed)); 
//			//Robot.driveTrain.m_rightMotors.set(((-output + angle) * speed)); 
		}
	}

	// Command is finished when average distance = total distance or command
	// times out
	protected boolean isFinished() {
		return Robot.driveTrain.getLeftEncoderDistance() == pathLenght || isTimedOut();
	}

	// At the end, stop drive motors
	protected void end() {
		Robot.driveTrain.stopRobot();
		recordPathingData1(timelist,tEncoderValueslist1,"/home/lvuser/theory6TimeList1.txt","/home/lvuser/tEncoderValueslist1.txt");
    	recordPathingData1(timelist,tEncoderValueslist2,"/home/lvuser/theory6TimeList2.txt","/home/lvuser/tEncoderValueslist2.txt");
		recordPathingData1(timelist,tCurveLenghtValuesList1,"/home/lvuser/theory6TimeList1.txt","/home/lvuser/tCurvelenghtlist1.txt");
    	recordPathingData1(timelist,tCurveLenghtValuesList2,"/home/lvuser/theory6TimeList2.txt","/home/lvuser/tCurvelenghtlist2.txt");
	}

	protected void interrupted() {
	}
}
