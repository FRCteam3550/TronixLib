package org.usfirst.frc3550.Julius2018.commands;

import org.usfirst.frc3550.Julius2018.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Speed2Command extends Command {

    public Speed2Command() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.gearsHandler);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	//setTimeout(2);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.gearsHandler.slowDown();
    	SmartDashboard.putBoolean("isLowSpeed", true);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
