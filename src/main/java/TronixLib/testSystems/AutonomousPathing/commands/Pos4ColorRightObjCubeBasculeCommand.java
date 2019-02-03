package org.usfirst.frc3550.Julius2018.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc3550.Julius2018.Robot;
import org.usfirst.frc3550.Julius2018.util.*;

/**
 *
 */
public class Pos4ColorRightObjCubeBasculeCommand extends CommandGroup {

    public Pos4ColorRightObjCubeBasculeCommand() {
    	
    	/*addSequential(new AutoDriveDistanceCommand(Robot.fieldMesures.LENGTHONE, 1)); // 0:Backward / 1:Forward
    	addSequential(new AutoDriveRotateCommand(Robot.fieldMesures.ROTATEANGLE,   0));    // 0:Left     / 1:Right
    	addSequential(new AutoDriveDistanceTimedCommand(Robot.fieldMesures.LENGHTTWO, 1, 3)); // 0:Backward / 1:Forward
    	*///addSequential(new OpenPince());
    	addSequential(new AutoDriveDistanceCommand(Robot.fieldMesures.P4R_LengthOne, 1)); // 0:Backward / 1:Forward
    	addSequential(new AutoDriveRotateCommand(Robot.fieldMesures.ROTATEANGLE,   0));    // 0:Left     / 1:Right
    	addSequential(new AutoDriveDistanceTimedCommand(Robot.fieldMesures.P4R_LengthTwo, 1, 3)); // 0:Backward / 1:Forward
    	
    	addSequential(new DropCubeCommand());
    }
}
