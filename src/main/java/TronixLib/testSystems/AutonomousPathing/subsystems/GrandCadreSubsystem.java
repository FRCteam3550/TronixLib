package org.usfirst.frc3550.Julius2018.subsystems;

import org.usfirst.frc3550.Julius2018.RobotMap;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class GrandCadreSubsystem extends Subsystem {
	
	private final SpeedController m_Chain = RobotMap.ChainMotor;
	private final DigitalInput m_GrandCadreUp = RobotMap.GrandCadreUp;
	private final DigitalInput m_GrandCadreDown = RobotMap.GrandCadreDown;
	private final DoubleSolenoid Leds = RobotMap.Leds;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	public GrandCadreSubsystem() {
		 
		addChild("GrandCadre SensorUp", m_GrandCadreUp);
		addChild("GrandCadre SensorDown", m_GrandCadreDown);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void setGrandCadreUp(){
    	m_Chain.set(0.95);
    }
    
    public void setGrandCadreDown(){
    	m_Chain.set(-0.95);
    }
    
    public void stop(){
    	m_Chain.set(0);
    }
    
    public boolean GrandCadreIsUp(){
    	return 	  !m_GrandCadreUp.get();
    }
    
    public boolean GrandCadreIsDown(){
    	return !m_GrandCadreDown.get();
    }
    
    public void setManual(double speed){
    	m_Chain.set(speed);
    }
    //public void LedOn( ) {
    	//Leds.set(DoubleSolenoid.Value.kForward);
   // }
    public void LedOff( ) {
    	Leds.set(DoubleSolenoid.Value.kReverse);
    }
}

