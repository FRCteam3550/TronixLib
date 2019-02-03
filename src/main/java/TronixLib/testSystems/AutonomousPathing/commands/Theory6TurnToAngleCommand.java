package org.usfirst.frc3550.Julius2018.commands;

import org.usfirst.frc3550.Julius2018.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Theory6TurnToAngleCommand extends Command {

	
	private double speed = 0.0;
	private double angle = 0.0;
	private double epsilon = 0.0;
	
    public Theory6TurnToAngleCommand(double angle, double speed, double epsilon) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    	this.speed = speed;
    	this.angle = angle;
    	this.epsilon = epsilon;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.getAhrs().reset();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double error = angle - Robot.driveTrain.getAhrs().getAngle();
    	
    	//SmartDashboard.putNumber("setspeed", setSpeed);
    	SmartDashboard.putNumber("SetPointPIDAngleTurn :", angle);
    	SmartDashboard.putNumber("RawPIDAngleTurn :" ,Robot.driveTrain.getAhrs().getYaw());
    	SmartDashboard.putNumber("ErrorPIDAngleTurn: ", error);
    	//Robot.driveTrain.tankDrive(setSpeed, -setSpeed);
    	//if(Robot.oi.getPilotStick().getRawButton(9)) {
    	Robot.driveTrain.turnDrive(angle, speed,epsilon);
    	//}
    	//else
    	//	Robot.driveTrain.arcadeDrive(-Robot.oi.getPilotStick().getY(),Robot.oi.getPilotStick().getZ());
    	SmartDashboard.putNumber ("TurntoAngle", Robot.driveTrain.getAhrs().getYaw());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return false;
      //  return Math.abs(angle - Robot.driveTrain.getAhrs().getYaw()) < 0.25;
       
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("TurnToAngleDONE");
    	Robot.driveTrain.tankDrive(0,0);
    	//Robot.driveTrain.getAhrs().reset();
    	SmartDashboard.putNumber("FINALANGLE:" ,Robot.driveTrain.getAhrs().getYaw());
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
