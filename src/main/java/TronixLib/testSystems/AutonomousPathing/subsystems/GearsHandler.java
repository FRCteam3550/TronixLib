package org.usfirst.frc3550.Julius2018.subsystems;

import java.nio.channels.FileChannel;

import org.usfirst.frc3550.Julius2018.RobotMap;
import org.usfirst.frc3550.Julius2018.commands.Speed2Command;

//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class GearsHandler extends Subsystem {

	    private final DoubleSolenoid LeftMotorSolenoid = RobotMap.LeftMotorSolenoid;
	   // private final DoubleSolenoid RightMotorSolenoid = RobotMap.RightMotorSolenoid;
	    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new TankDriveWithJoystick());
       // setDefaultCommand(new ArcadeDrive());
    	setDefaultCommand(new Speed2Command());
    }
    
  
	public void speedUp() {
		LeftMotorSolenoid.set(DoubleSolenoid.Value.kForward);
		//RightMotorSolenoid.set(Value.kReverse);
		
		
	}
	public void slowDown(){
		LeftMotorSolenoid.set(DoubleSolenoid.Value.kReverse);
		//RightMotorSolenoid.set(Value.kForward);
		
		
	}
}
	


