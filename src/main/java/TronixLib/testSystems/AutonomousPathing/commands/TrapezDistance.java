package org.usfirst.frc3550.Julius2018.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc3550.Julius2018.util.*;
import org.usfirst.frc3550.Julius2018.Robot;

/**
 *
 */
public class TrapezDistance extends Command {
	
	double initialTime = System.currentTimeMillis();
	TrapezProfile profile;
	MotionProfileFollower follower;

    public TrapezDistance() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	profile = new TrapezProfile(0.1, 1, 1, 0);
    	follower = new MotionProfileFollower(1);
    	follower.setProfile(profile.getGeneratedProfile());
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double currentTimeDelta = (System.currentTimeMillis()/1000)-(initialTime/1000);
    	double motorOutput = follower.getMotorOutput(currentTimeDelta);
    	Robot.driveTrain.arcadeDrive(motorOutput, 0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return follower.isFinishedTime((System.currentTimeMillis()/1000)-(initialTime/1000));
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
