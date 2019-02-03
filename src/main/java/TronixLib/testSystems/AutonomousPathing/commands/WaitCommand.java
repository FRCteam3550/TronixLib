package org.usfirst.frc3550.Julius2018.commands;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class WaitCommand extends Command {
	double time;
    public WaitCommand(double time) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	time = time;
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	setTimeout(time);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
