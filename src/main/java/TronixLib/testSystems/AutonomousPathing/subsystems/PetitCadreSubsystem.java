package org.usfirst.frc3550.Julius2018.subsystems;

import org.usfirst.frc3550.Julius2018.RobotMap;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class PetitCadreSubsystem extends Subsystem {
	
	private final SpeedController m_Poulie = RobotMap.PoulieMotor;
	private final DigitalInput m_PetitCadreUp = RobotMap.PetitCadreUp;
	private final DigitalInput m_PetitCadreDown = RobotMap.PetitCadreDown;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	public PetitCadreSubsystem() {
		 
		addChild("PetitCadre SensorUp", m_PetitCadreUp);
		addChild("PetitCadre SensorDown", m_PetitCadreDown);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void setPetitCadreUp(){
    	m_Poulie.set(1);
    }
    
    public void setPetitCadreDown(){
    	m_Poulie.set(-1);
    }
    
    public void stop(){
    	m_Poulie.set(0);
    }
    
    public boolean PetitCadreIsUp(){
    	return !m_PetitCadreUp.get();
    }
    
    public boolean PetitCadreIsDown(){
    	return !m_PetitCadreDown.get();
    }
    
    
}

