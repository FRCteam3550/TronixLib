package org.usfirst.frc3550.Julius2018.commands;

import org.usfirst.frc3550.Julius2018.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Pos1ColorRightObjCubeBasculeCommand extends CommandGroup {

	public Pos1ColorRightObjCubeBasculeCommand() {
   	  	addSequential(new AutoDriveDistanceCommand(Robot.fieldMesures.P1R_LengthOne, 1)); // 0:Backward / 1:Forward
		addSequential(new AutoDriveRotateCommand(Robot.fieldMesures.ROTATEANGLE, 1));    // 0:Left     / 1:Right
		addSequential(new AutoDriveDistanceCommand(Robot.fieldMesures.P1R_LengthTwo,  1)); // 0:Backward / 1:Forward
    	addSequential(new AutoDriveRotateCommand(Robot.fieldMesures.ROTATEANGLE, 1));
    	addSequential(new AutoDriveDistanceTimedCommand(Robot.fieldMesures.P1R_LengthThree, 1,0.5)); // 0:Backward / 1:Forward
    	addSequential(new DropCubeCommand());
    }
}

