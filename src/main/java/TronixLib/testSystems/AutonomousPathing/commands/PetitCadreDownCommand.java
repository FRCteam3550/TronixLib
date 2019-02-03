package org.usfirst.frc3550.Julius2018.commands;

import org.usfirst.frc3550.Julius2018.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PetitCadreDownCommand extends Command {

    public PetitCadreDownCommand() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.PetitCadreSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	//setTimeout(5);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
//   	if (Robot.PetitCadreSubsystem.PetitCadreIsDown()) {
//   		Robot.PetitCadreSubsystem.stop();
//    	}
//   	else Robot.PetitCadreSubsystem.PetitCadreIsDown();
    	Robot.PetitCadreSubsystem.setPetitCadreDown();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
      //  return isTimedOut();
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.PetitCadreSubsystem.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
