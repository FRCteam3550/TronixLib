package org.usfirst.frc3550.Julius2018.commands;

import org.usfirst.frc3550.Julius2018.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Pos4ColorLeftObjCubeBasculeCommand extends CommandGroup {

	public Pos4ColorLeftObjCubeBasculeCommand() {
    	addSequential(new AutoDriveDistanceCommand(Robot.fieldMesures.P4L_LengthOne, 1)); // 0:Backward / 1:Forward
    	addSequential(new AutoDriveRotateCommand(Robot.fieldMesures.ROTATEANGLE,   0));    // 0:Left     / 1:Right
    	addSequential(new AutoDriveDistanceCommand(Robot.fieldMesures.P4L_LengthTwo, 1)); // 0:Backward / 1:Forward
    	addSequential(new WaitCommand(2));
    	addSequential(new AutoDriveRotateCommand(Robot.fieldMesures.ROTATEANGLE,   0));
    	//addSequential(new AutoDriveDistanceCommand(Robot.fieldMesures.P4L_LengthThree, 1));
    	addSequential(new DropCubeCommand());
    }
}
