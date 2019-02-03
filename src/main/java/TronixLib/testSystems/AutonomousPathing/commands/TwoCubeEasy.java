package org.usfirst.frc3550.Julius2018.commands;

import java.util.function.Function;

//import org.usfirst.frc.team340.robot.commands.claw.ClawAcquireCube;
//import org.usfirst.frc.team340.robot.commands.claw.ClawShootScore;
//import org.usfirst.frc.team340.robot.commands.elevator.ElevatorGoToBottom;
//import org.usfirst.frc.team340.robot.commands.elevator.ElevatorGoToPosition;
//import org.usfirst.frc.team340.robot.commands.manual.ManualClawClose;
//import org.usfirst.frc.team340.robot.commands.manual.ManualClawOpen;
//import org.usfirst.frc.team340.robot.commands.manual.ManualClawWheelsIn;
//import org.usfirst.frc.team340.robot.commands.manual.ManualClawWheelsStop;
import org.usfirst.frc3550.Julius2018.commands.pathing.Path;
import org.usfirst.frc3550.Julius2018.commands.pathing.Paths;
import org.usfirst.frc3550.Julius2018.commands.pathing.RunPath;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class TwoCubeEasy extends CommandGroup {
	private final Function<Double, Double> speedFast = x -> {
		if(x < 0.1) {
			return 0.3;
		} else if (x < 0.85) {
			return 0.6969;
		} else {
			return 0.25;
		}
	};
	private final Function<Double, Double> speedSlow = x -> {
		if(x < 0.3) {
			return 0.3;
		} else if (x < 0.75) {
			return 0.5;
		} else {
			return 0.25;
		}
	};
	
    public TwoCubeEasy(Path travelPath, Path finishPath, Path secondCube, double turnAngle, double scaleSpeed, double switchSpeed) {
    	addSequential(new SingleCube(travelPath, finishPath, 3000, scaleSpeed), 11); //original
//    	/addSequential(new ElevatorGoToBottom(), 2);
       // addSequential(new RunPath(Paths.straightLength(15), -0.5), 1.5); //original
    	//addSequential(new TurnToAngle(turnAngle, 0.50), 1.5);  //original
    	//addSequential(new AutoDriveRotateCommand(turnAngle, 0)); //    /	addSequential(new WaitCommand(0.25));
    	///addSequential(new ManualClawOpen(), 0.5);
    	///addParallel(new ManualClawWheelsIn());
        addSequential(new RunPath(secondCube, 0.7), 8); //original
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
