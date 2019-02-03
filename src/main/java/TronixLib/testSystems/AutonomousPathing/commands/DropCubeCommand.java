package org.usfirst.frc3550.Julius2018.commands;

import org.usfirst.frc3550.Julius2018.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DropCubeCommand extends Command {

    public DropCubeCommand() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.pinceSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.pinceSubsystem.ouvrePince(); // ouvrir dans le code 2018
    	setTimeout(3);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	Robot.pinceSubsystem.dropCube();
//    	if(Robot.pinceSubsystem.IsCubeHere()) 
//    	{
//    		if(Robot.pinceSubsystem.readyCondamne()){  
//    	      Robot.pinceSubsystem.pickupCube();
//    	      }
//    		else
//    			{
//    	    	 Robot.pinceSubsystem.ouvrePince(); //fermer dans le robot de 2018
//    			}
//    }
//    	else{
//    		Robot.pinceSubsystem.Stop();
//    		}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.pinceSubsystem.Stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	//isTimedOut();
    	end();
    }
}
