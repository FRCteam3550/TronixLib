package org.usfirst.frc3550.Julius2018.commands;

import org.usfirst.frc3550.Julius2018.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class TurnToAngle extends Command {

	private final double P = 50; //10
	private double speed = 0.0;
	private double angle = 0.0;
	
    public TurnToAngle(double angle, double speed) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    	this.speed = speed;
    	this.angle = angle;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.getAhrs().reset();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double error = angle - Robot.driveTrain.getAhrs().getAngle();
    	double setSpeed = error / (P / Math.abs(speed));
    	if (setSpeed > 1) {
    		setSpeed = 0.99;
    	}
    	if (setSpeed < -1) {
    		setSpeed = -0.99;
    	}
    	System.out.println("setspeed: " +setSpeed );
    	//SmartDashboard.putNumber("setspeed", setSpeed);
    	System.out.println("SetPointPIDAngleTurn :" + angle);
    	System.out.println ("RawPIDAngleTurn :" +Robot.driveTrain.getAhrs().getYaw());
    	System.out.println ("ErrorPIDAngleTurn: "+ error);
    	//Robot.driveTrain.tankDrive(setSpeed, -setSpeed);
    	Robot.driveTrain.curvatureDrive(0, (error),true);
    	SmartDashboard.putNumber ("TurntoAngle", Robot.driveTrain.getAhrs().getYaw());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(angle - Robot.driveTrain.getAhrs().getYaw()) < 1;
       
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("TurnToAngleDONE");
    	Robot.driveTrain.tankDrive(0,0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.driveTrain.tankDrive(0,0);
    }
}
