package org.usfirst.frc3550.Julius2018.commands;

//import org.omg.CORBA.TIMEOUT;
import org.usfirst.frc3550.Julius2018.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PickupCubeCommand extends Command {

    public PickupCubeCommand() {
        // Use requires() here to declare subsystem dependencies
         requires(Robot.pinceSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.pinceSubsystem.ouvrePince();
    	setTimeout(3);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//Robot.pinceSubsystem.pickupCube();
    	if(!Robot.pinceSubsystem.IsCubeHere()) 
    	{
    		if(!Robot.pinceSubsystem.readyCondamne()){  
    	      Robot.pinceSubsystem.pickupCube();
    	      }
    		else
    			{
    	    	 Robot.pinceSubsystem.fermePince(); //fermer dans le robot de 2018
    			}
    }
    	else{
    		Robot.pinceSubsystem.Stop();
    		}
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
       return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.pinceSubsystem.Stop();
    	//new HoldCubeCommand();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	//isTimedOut();
    	end();
    }
}
