package org.usfirst.frc3550.Julius2018.commands;

import org.usfirst.frc3550.Julius2018.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveStraightCommandTheory6 extends Command {

    public DriveStraightCommandTheory6() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.resetEncoders();
    	Robot.driveTrain.getAhrs().reset();
      Robot.driveTrain.stopRobot();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//Robot.driveTrain.arcadeDrive(-Robot.oi.getPilotStick().getY(), Robot.oi.getPilotStick().getZ(),true); //after Finger Lakes
    	//Robot.driveTrain.curvatureDrive(-Robot.oi.getPilotStick().getY(), Robot.oi.getPilotStick().getZ(),true);
    	Robot.driveTrain.driveStraight(0, 0.7, -90, 0.5); // Finger Lakes settings
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveTrain.stopRobot();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
