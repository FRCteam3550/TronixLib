package org.usfirst.frc3550.Julius2018.subsystems;

import org.usfirst.frc3550.Julius2018.RobotMap;
import org.usfirst.frc3550.Julius2018.commands.StopGrimpeurCommand;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Grimpeur extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private final SpeedController m_Grimpeur = RobotMap.m_Grimpeur;

    public void initDefaultCommand() {
        //Set the default command for a subsystem here.
        setDefaultCommand(new StopGrimpeurCommand());
    	
    	
    }
    public void climbUp() {
    m_Grimpeur.set(0.7);
    
}
    public void climbDown() {
    	m_Grimpeur.set(-0.7);
    }
    public void Stop() {
    	m_Grimpeur.set(0);
    }
}

