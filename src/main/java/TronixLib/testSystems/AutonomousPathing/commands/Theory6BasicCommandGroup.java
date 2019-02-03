package org.usfirst.frc3550.Julius2018.commands;

import org.usfirst.frc3550.Julius2018.theory6.pathing.Paths6.FROM_RIGHT;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Theory6BasicCommandGroup extends CommandGroup {

    public Theory6BasicCommandGroup() {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    	
    	addSequential(new Theory6driveSraightCommand(1, 0.9, 0, 0.25), 1);
    	addSequential(new  RunPathTheory6(FROM_RIGHT.START_POINT,FROM_RIGHT.CONTROL_POINT_ONE,FROM_RIGHT.CONTROL_POINT_TWO,FROM_RIGHT.END_POINT,10,0.8, false));
    	//addSequential(new Theory6driveSraightCommand(56, 0.7, 0, 0.25));
    }
}
