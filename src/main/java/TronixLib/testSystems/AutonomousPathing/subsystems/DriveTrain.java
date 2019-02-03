package org.usfirst.frc3550.Julius2018.subsystems;

import org.usfirst.frc3550.Julius2018.RobotMap;
import org.usfirst.frc3550.Julius2018.commands.ArcadeDrive;
import org.usfirst.frc3550.Julius2018.theory6.pathing.NumberConstants;
import org.usfirst.frc3550.Julius2018.theory6.pathing.PIDController;

//import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 *
 */
public class DriveTrain extends Subsystem {
	
    private final SpeedController m_RearLeft = RobotMap.RearLeft;
    private final SpeedController m_RearRight = RobotMap.RearRight;
    private final SpeedController m_FrontLeft = RobotMap.FrontLeft;
    private final SpeedController m_FrontRight = RobotMap.FrontRight;
    
    public final  SpeedControllerGroup m_leftMotors;
	public  final  SpeedControllerGroup m_rightMotors;
	private final  DifferentialDrive m_robotDrive;
    
	//private final DifferentialDrive m_robotDrive = RobotMap.m_drive;
	
	private final Encoder m_LeftEncoder = RobotMap.m_leftEncoder;
	private final Encoder m_RightEncoder = RobotMap.m_rightEncoder;
	private final DoubleSolenoid m_LeftMotorSolenoid = RobotMap.LeftMotorSolenoid;
	private final DoubleSolenoid m_Leds       = RobotMap.mLeds;
	private AHRS ahrs;
	
	/** The drive PID controller. */
	public PIDController drivePID;

	/** The gyro PID conteroller. */
	public PIDController gyroPID;
	
	
	//private TrapezSpeedProfile trapez;

	public DriveTrain() {
		try {
			/* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
			/* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
			/* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
			ahrs = new AHRS(SPI.Port.kMXP); 
			//ahrs = new AHRS(SerialPort.Port.kUSB1); 
		} catch (RuntimeException ex ) {
			DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
		}
		
		m_FrontLeft.setInverted(false);
		m_RearRight.setInverted(true);
		m_FrontRight.setInverted(false);
		
		m_leftMotors = new SpeedControllerGroup(m_FrontLeft, m_RearLeft);
		m_rightMotors = new SpeedControllerGroup(m_FrontRight, m_RearRight);
		
		m_leftMotors.setInverted(true);
		
		m_robotDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);
		
		m_robotDrive.setSafetyEnabled(true);
		m_robotDrive.setExpiration(0.1); //replaces 0.1 by 0.2 to suppress the DifferentialDrive output not updated often enough
		m_robotDrive.setMaxOutput(1.0);
	
		m_LeftEncoder.setMaxPeriod(.1);
		m_LeftEncoder.setMinRate(0.1);
		m_LeftEncoder.setDistancePerPulse(0.0100); //0.0166 robot competition //robot de tests 0.0100
		m_LeftEncoder.setReverseDirection(true);
		m_LeftEncoder.setSamplesToAverage(7);

		m_RightEncoder.setMaxPeriod(.1);
		m_RightEncoder.setMinRate(0.1);
		m_RightEncoder.setDistancePerPulse(0.0100);// pi*6inches/(4.41*256) //robot tests 0.0100
		m_RightEncoder.setReverseDirection(false);
		m_RightEncoder.setSamplesToAverage(7);
		//
		// Initialize PID controllers
		drivePID = new PIDController(NumberConstants.pDrive, NumberConstants.iDrive, NumberConstants.dDrive);
		gyroPID = new PIDController(NumberConstants.pGyro, NumberConstants.iGyro, NumberConstants.dGyro);
		
		//drivePID = new PIDController(5, 0, 0);
		//gyroPID = new PIDController(2, 0, 0);
		
		System.out.println("expiration time"+ m_robotDrive.getExpiration());
		//		addChild("Right Encoder", RightEncoder);
		addChild("Left Encoder", m_LeftEncoder);
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new TankDriveWithJoystick());
		setDefaultCommand(new ArcadeDrive());
	}
	/**
	 * Prevent robot from moving.
	 */
	public void stopRobot() {
		m_robotDrive.stopMotor();
	}

	/**
	 * Tank drive using individual joystick axes.
	 *
	 * @param leftAxis
	 *            Left sides value
	 * @param rightAxis
	 *            Right sides value
	 */
	public void tankDrive(double leftAxis, double rightAxis) {
		m_robotDrive.tankDrive(leftAxis, rightAxis);
	}

	/**
	 * Arcade drive using individual joystick axes.
	 *
	 * @param leftAxis
	 *            Left sides value
	 * @param rightAxis
	 *            Right sides value
	 */
	public void arcadeDrive(Joystick stick) {
		m_robotDrive.arcadeDrive(-stick.getY(), stick.getX());//(-)for the 2018 robot//
		//robotDrive41.arcadeDrive(stick.getY(), -stick.getX());//for the 2017 robot
	}
	public void arcadeDrive(double speed, double rotation) {
		m_robotDrive.arcadeDrive(speed, rotation);
	}

	public void arcadeDrive(double speed, double rotation, boolean sensitivity) {
		m_robotDrive.arcadeDrive(speed, rotation, sensitivity);
	}

	public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
		m_robotDrive.curvatureDrive(xSpeed, zRotation, isQuickTurn);
	}

	public void driveStraight(double setPoint, double speed, double setAngle, double epsilon) {
		//drivePID = new PIDController(NumberConstants.pDrive, NumberConstants.iDrive, NumberConstants.dDrive);
		//gyroPID = new PIDController(NumberConstants.pGyro, NumberConstants.iGyro, NumberConstants.dGyro);
		double output = drivePID.calcPIDDrive(setPoint, getLeftEncoderDistance(), epsilon);
		//System.out.println("output = "+ output);
		double angle = gyroPID.calcPID(setAngle, getAhrs().getYaw(), epsilon);
		//System.out.println("angle = "+ angle);
		//System.out.println("leftMotorsSpeed = "+ ((output + angle) * speed));
		//System.out.println("rightMotorsSpeed = "+ setPoint);
		//m_leftMotors.set(((output + angle) * speed)); 
		//m_rightMotors.set(((-output + angle) * speed)); 
		m_leftMotors.set(((output + angle) * speed)); 
		m_rightMotors.set(((-output + angle) * speed)); 
		//tankDrive(((output + angle) * speed), ((-output + angle) * speed));
	}
	
	public void driveAngle(double setAngle, double speed) {
		//drivePID = new PIDController(NumberConstants.pDrive, NumberConstants.iDrive, NumberConstants.dDrive);
		//gyroPID = new PIDController(NumberConstants.pGyro, NumberConstants.iGyro, NumberConstants.dGyro);
		//double output = drivePID.calcPIDDrive(setPoint, getLeftEncoderDistance(), epsilon);
		//System.out.println("output = "+ output);
		double angle = gyroPID.calcPID(setAngle, getAhrs().getYaw(), 1);
		//System.out.println("angle = "+ angle);
		//System.out.println("leftMotorsSpeed = "+ ((output + angle) * speed));
		//System.out.println("rightMotorsSpeed = "+ setPoint);
		m_leftMotors.set(speed + angle); 
		m_rightMotors.set(-speed + angle); 
		//tankDrive(((output + angle) * speed), ((-output + angle) * speed));
	}
	
	public void turnDrive(double setAngle, Double speed, double epsilon) {
		//drivePID = new PIDController(NumberConstants.pDrive, NumberConstants.iDrive, NumberConstants.dDrive);
		//gyroPID = new PIDController(NumberConstants.pGyro, NumberConstants.iGyro, NumberConstants.dGyro);
		//double output = drivePID.calcPIDDrive(setPoint, getLeftEncoderDistance(), epsilon);
		//System.out.println("output = "+ output);
		double angle = gyroPID.calcPID(setAngle, getAhrs().getYaw(), epsilon);
		//System.out.println("angle = "+ angle);
		//System.out.println("leftMotorsSpeed = "+ ((output + angle) * speed));
		//System.out.println("rightMotorsSpeed = "+ setPoint);
		m_leftMotors.set(angle * speed); 
		m_rightMotors.set(angle * speed); 
		//tankDrive(((output + angle) * speed), ((-output + angle) * speed));
	}
	
	/**
	 * Interface to Trapez.
	 */
	//	public void SetDistance(int distance, double vmax, double a) {
	//		trapez.SetDistance(distance, vmax, a);
	//	}
	//	
	//	public double DistanceMove() {
	//		double curSpeed = trapez.DistanceMove();
	//		robotDrive41.arcadeDrive(curSpeed, 0.0);
	//	}
	//	
	//	public boolean isDistanceDone() {
	//		return trapez.isDistanceDone();
	//	}
	
	public void resetEncoders() {
		m_LeftEncoder.reset();
		m_RightEncoder.reset();
	}

	public Encoder getLeftEncoder() {
		return m_LeftEncoder;
	}
	public Encoder getRightEncoder() {
		return m_RightEncoder;
	}
	public double getLeftEncoderDistance() {
		return m_LeftEncoder.getDistance();
	}

	public double getRightEncoderDistance() {
		return m_RightEncoder.getDistance();
	}

	//public double getRightDistance() {
	//	return RightEncoder.getDistance();
	//}

//	public double getLeftEncoderRate() {
//		return m_LeftEncoder.getDistance();
//	}

	//public double getRightRate() {
	//return RightEncoder.getDistance();
	//}

	public double getDistance() {
	return (m_LeftEncoder.getDistance() + m_RightEncoder.getDistance()) / 2;
	}

	public void speedUpDoubleGear() {
		m_LeftMotorSolenoid.set(DoubleSolenoid.Value.kForward);
	}

	public void slowDownDoubleGear() {
		m_LeftMotorSolenoid.set(DoubleSolenoid.Value.kReverse);
	}

	public AHRS getAhrs()  {
		return ahrs;

	}

	public void disableAllOnBoard(){
		resetEncoders();
		getAhrs().reset();
		stopRobot();
	}

	public void LedsOn() {
		m_Leds.set(DoubleSolenoid.Value.kForward);
	}

	public void LedsOff() {
		m_Leds.set(DoubleSolenoid.Value.kReverse);
	}
}



