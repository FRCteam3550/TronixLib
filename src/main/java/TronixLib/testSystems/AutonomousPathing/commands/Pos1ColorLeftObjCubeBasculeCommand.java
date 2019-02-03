package org.usfirst.frc3550.Julius2018.commands;

import org.usfirst.frc3550.Julius2018.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Pos1ColorLeftObjCubeBasculeCommand extends CommandGroup {

    public Pos1ColorLeftObjCubeBasculeCommand() {
    	
    	/*addSequential(new AutoDriveDistanceCommand(Robot.fieldMesures.LENGTHONE, 1)); //distance:150|| 0:Backward / 1:Forward
    	addSequential(new AutoDriveRotateCommand(Robot.fieldMesures.ROTATEANGLE, 1));    // 0:Left     / 1:Right
    	addSequential(new AutoDriveDistanceTimedCommand(Robot.fieldMesures.LENGHTTWO,1,3)); //distance:20|| 0:Backward / 1:Forward//time
    	addSequential(new DropCubeCommand());*/
    	addSequential(new AutoDriveDistanceCommand(Robot.fieldMesures.P1L_LengthOne, 1)); //distance:150|| 0:Backward / 1:Forward
    	addSequential(new AutoDriveRotateCommand(Robot.fieldMesures.ROTATEANGLE, 1));    // 0:Left     / 1:Right
    	addSequential(new AutoDriveDistanceTimedCommand(Robot.fieldMesures.P1L_LengthTwo,1,3)); //distance:20|| 0:Backward / 1:Forward//time
    	addSequential(new DropCubeCommand());
    	//addSequential(new PickupCubeCommand());
    }
}
