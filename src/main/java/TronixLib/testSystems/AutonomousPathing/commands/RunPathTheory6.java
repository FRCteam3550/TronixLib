package org.usfirst.frc3550.Julius2018.commands;


import org.usfirst.frc3550.Julius2018.theory6.pathing.*;

//import org.usfirst.frc.team340.robot.commands.claw.ClawAcquireCube;
//import org.usfirst.frc.team340.robot.commands.claw.ClawShootScore;
//import org.usfirst.frc.team340.robot.commands.elevator.ElevatorGoToBottom;
//import org.usfirst.frc.team340.robot.commands.elevator.ElevatorGoToPosition;
//import org.usfirst.frc.team340.robot.commands.manual.ManualClawClose;
//import org.usfirst.frc.team340.robot.commands.manual.ManualClawOpen;
//import org.usfirst.frc.team340.robot.commands.manual.ManualClawWheelsIn;
//import org.usfirst.frc.team340.robot.commands.manual.ManualClawWheelsStop;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class RunPathTheory6 extends CommandGroup {
	
    public RunPathTheory6(Point startPoint, Point controlePoint1 , Point controlePoint2, Point endPoint, double timeOut, double speed, boolean reverse) {
    	addSequential(new DrivePath(startPoint, controlePoint1, controlePoint2, endPoint, timeOut, speed, reverse));
//    	/addSequential(new ElevatorGoToBottom(), 2);
       // addSequential(new RunPath(Paths.straightLength(18), -0.5), 1.5);
    	//addSequential(new TurnToAngle(turnAngle, 0.50));
    	//addSequential(new AutoDriveRotateCommand(turnAngle, 0)); //    /	addSequential(new WaitCommand(0.25));
    	///addSequential(new ManualClawOpen(), 0.5);
    	///addParallel(new ManualClawWheelsIn());
    //	addSequential(new RunPath(secondCube, 0.7), 3); test on Sunday -Regis
    	///addSequential(new ManualClawClose(), 0.5);
    	///addSequential(new ManualClawWheelsStop(), 0.5);
    	///addSequential(new ClawAcquireCube(), 1);
    	///addSequential(new WaitCommand(0.25));
    	//addSequential(new RunPath(Paths.straightLength(3), -0.3), 0.2);
    	///addSequential(new ElevatorGoToPosition(969), 1.0);
    	//addSequential(new RunPath(Paths.straightLength(5), 0.7));
    	////addSequential(new ClawShootScore(switchSpeed), 1);
    }
}
