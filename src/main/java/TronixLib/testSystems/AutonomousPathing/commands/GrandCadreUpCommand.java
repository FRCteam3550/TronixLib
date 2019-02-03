package org.usfirst.frc3550.Julius2018.commands;

import org.usfirst.frc3550.Julius2018.Robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class GrandCadreUpCommand extends Command {

    public GrandCadreUpCommand() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.GrandCadreSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	//setTimeout(4);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
//    	if (!Robot.GrandCadreSubsystem.GrandCadreIsUp()) {
//    		Robot.GrandCadreSubsystem.setGrandCadreUp();
//
//    	}
//    	else 
//    	Robot.GrandCadreSubsystem.stop();
    	Robot.GrandCadreSubsystem.setGrandCadreUp();
    	//Robot.GrandCadreSubsystem.LedOn();
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
      //  return isTimedOut();
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.GrandCadreSubsystem.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
