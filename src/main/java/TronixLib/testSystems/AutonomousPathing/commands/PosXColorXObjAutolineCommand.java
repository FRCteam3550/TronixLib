package org.usfirst.frc3550.Julius2018.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class PosXColorXObjAutolineCommand extends CommandGroup {

    public PosXColorXObjAutolineCommand() {    	
        addSequential(new AutoDriveDistanceCommand(120, 1)); // 1 = Forward
    	
    }
}
