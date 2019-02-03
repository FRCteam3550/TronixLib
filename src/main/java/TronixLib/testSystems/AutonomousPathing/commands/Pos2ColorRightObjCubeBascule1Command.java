package org.usfirst.frc3550.Julius2018.commands;

import org.usfirst.frc3550.Julius2018.util.FieldMesures;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Pos2ColorRightObjCubeBascule1Command extends CommandGroup {

	public Pos2ColorRightObjCubeBascule1Command() {
    	/*addSequential(new AutoDriveDistanceCommand(30, 1)); // 0:Backward / 1:Forward
    	addSequential(new AutoDriveRotateCommand(90,   1));    // 0:Left     / 1:Right
    	addSequential(new AutoDriveDistanceCommand(45, 1)); // 0:Backward / 1:Forward
    	addSequential(new AutoDriveRotateCommand(90,   0));
    	addSequential(new AutoDriveDistanceTimedCommand(65, 1, 3));*/
		addSequential(new AutoDriveDistanceCommand(FieldMesures.P2R_LengthOne, 1)); // 0:Backward / 1:Forward
    	addSequential(new AutoDriveRotateCommand(FieldMesures.ROTATEANGLE,   1));    // 0:Left     / 1:Right
    	addSequential(new AutoDriveDistanceCommand(FieldMesures.P2R_LengthTwo, 1)); // 0:Backward / 1:Forward
    	addSequential(new AutoDriveRotateCommand(FieldMesures.ROTATEANGLE,   0));
    	addSequential(new AutoDriveDistanceTimedCommand(FieldMesures.P2R_LengthThree, 1, 3));
    	addSequential(new DropCubeCommand());
    }
}
