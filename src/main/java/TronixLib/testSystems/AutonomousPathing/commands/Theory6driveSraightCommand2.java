package org.usfirst.frc3550.Julius2018.commands;

import org.usfirst.frc3550.Julius2018.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Theory6driveSraightCommand2 extends Command {

	private double targetDistance = 0.0;
	private double speed = 0.0;
	private double targetAngle = 0.0;
	private double epsilon = 0.0;
	
    public Theory6driveSraightCommand2(double targetDistance, double speed, double targetAngle, double epsilon) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	//requires(Robot.driveTrain);
    	this.targetDistance = targetDistance;
    	this.speed = speed;
    	this.targetAngle = targetAngle;
    	this.epsilon = epsilon;
    	requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.getAhrs().reset();
    	Robot.driveTrain.getLeftEncoder().reset();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double errorInDistance = targetDistance - Robot.driveTrain.getLeftEncoderDistance();
    	
    	SmartDashboard.putNumber("targetAngle", targetAngle);
    	SmartDashboard.putNumber("actualGyroReadings", Robot.driveTrain.getAhrs().getYaw());
    	SmartDashboard.putNumber("targetDistance :", targetDistance);
    	SmartDashboard.putNumber("RawPIDDistance :" ,Robot.driveTrain.getLeftEncoderDistance());
    	SmartDashboard.putNumber("ErrorPIDAngleTurn: ", errorInDistance);
    	//Robot.driveTrain.tankDrive(setSpeed, -setSpeed);
    	//if(Robot.oi.getPilotStick().getRawButton(9)) {
    	Robot.driveTrain.driveStraight(targetDistance, speed,targetAngle,epsilon);
    	//}
    	//else
    	//	Robot.driveTrain.arcadeDrive(-Robot.oi.getPilotStick().getY(),Robot.oi.getPilotStick().getZ());
    	SmartDashboard.putNumber ("afterPIDEncoderDistance", Robot.driveTrain.getLeftEncoderDistance());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
       return Math.abs(targetDistance - Robot.driveTrain.getLeftEncoderDistance()) < epsilon;
       
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("TurnToAngleDONE");
    	Robot.driveTrain.tankDrive(0,0);
    	Robot.driveTrain.getAhrs().reset();
    	Robot.driveTrain.getAhrs().resetDisplacement();
    	Robot.driveTrain.getLeftEncoder().reset();
    	SmartDashboard.putNumber("finalEncoderReadings:" ,Robot.driveTrain.getLeftEncoderDistance());
    	SmartDashboard.putNumber("finalGyroReadings:" ,Robot.driveTrain.getAhrs().getYaw());
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.driveTrain.getAhrs().reset();
    	Robot.driveTrain.getAhrs().reset();
    	Robot.driveTrain.getLeftEncoder().reset();
    }
}
