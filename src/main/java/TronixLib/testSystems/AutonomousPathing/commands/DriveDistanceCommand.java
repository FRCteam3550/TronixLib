package org.usfirst.frc3550.Julius2018.commands;

import org.usfirst.frc3550.Julius2018.Robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveDistanceCommand extends Command {

	
	private static final double kAngleSetpoint = 0.0;
	private static final double kPAngle =  0.0979; // propotional turning constant
	
	private double m_setpoint = 0;

	private static final double kP = 7; //

	// integral speed constant
	private static final double kI = 0.01; //0.018

	// derivative speed constant
	private static final double kD = 1; //1.5

	// derivative speed constant
	private static final double kF = 0;

	private double m_error = 0;
	double m_DistanceValue;
	//double m_SensValue;


	private PIDController m_EncoderPID; 

	private PIDOutput m_PIDOutput;


	// Angle in degre
	// Direction = 0: Left 1:Right
	public DriveDistanceCommand(double distance, int direction) {
		if(direction == 0){
		this.m_setpoint = -distance;
		//this.m_SensValue = -1;
		}
		else if(direction == 1){
			this.m_setpoint = Math.abs(distance);
			//this.m_SensValue = 1;
		}
		else {
			 this.m_setpoint = Math.abs(distance);
		    // this.m_SensValue = 1;
		     }
		// Use requires() here to declare subsystem dependencies
		requires(Robot.driveTrain);
		m_EncoderPID = new PIDController(kP,kI,kD,kF, Robot.driveTrain.getLeftEncoder(),  new MyPidOutput());

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		 Robot.driveTrain.getLeftEncoder().reset();
		 Robot.driveTrain.getAhrs().reset();
		//m_GyroPID.disable(); // begin PID control
		m_EncoderPID.setInputRange(-100000.0f, 100000.0f);
		//m_GyroPID.setOutputRange(-0.4, 0.4);
		m_EncoderPID.setAbsoluteTolerance(0.1f);
		// error = setpoint - Robot.drivetrain.getAhrs().getAngle();
		// Set setpoint of the pid controller
		SmartDashboard.putNumber("EncoderPIDSetPoint", m_setpoint);
		
		m_EncoderPID.setContinuous(true);
		m_EncoderPID.setSetpoint(m_setpoint);
		m_EncoderPID.enable(); // begin PID control
		m_EncoderPID.enable(); // begin PID control
       // setTimeout(1);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		// Set setpoint of the pid controller
		m_error = m_setpoint - Robot.driveTrain.getLeftEncoder().getDistance();
		SmartDashboard.putNumber("EncoderPID_ERROR", m_error);
		SmartDashboard.putNumber("EncoderPID_value", Robot.driveTrain.getLeftEncoder().getDistance());
		SmartDashboard.putNumber("EncoderPIDSetPointCorrection",-m_DistanceValue);
		double turningValue = (kAngleSetpoint - Robot.driveTrain.getAhrs().getAngle()) * kPAngle;
		// Invert the direction of the turn if we are going backwards
		turningValue = Math.copySign(turningValue, turningValue);
		SmartDashboard.putNumber("turningAngleCorrectionNax", Robot.driveTrain.getAhrs().getAngle());
		SmartDashboard.putNumber("turningAngleCorrection",turningValue);
		//Robot.driveTrain.arcadeDrive(0, -rotateValue); // robot de competition
		Robot.driveTrain.arcadeDrive(m_DistanceValue, turningValue); // robot de test
		
		//m_GyroPID.enable(); // begin PID control	
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		//return (m_GyroPID.onTarget() || isTimedOut());
		return (Math.abs(m_error)<1);
		//return ((Math.abs(error)<1) || isTimedOut());
	}

	// Called once after isFinished returns true
	protected void end() {
		//Robot.drivetrain.disableDriveTrain();
		Robot.driveTrain.stopRobot();
		m_EncoderPID.disable(); // begin PID control
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}

	private class MyPidOutput implements PIDOutput {
		@Override
		public void pidWrite(double output) {
			m_DistanceValue = output;
			if(m_DistanceValue > 0.7) //0.5 robot de competition
				m_DistanceValue = 0.7;
			else if(m_DistanceValue < -0.7)
				m_DistanceValue = -0.7;	
		}
	}
}
