


// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc3550.Julius2018;

import org.usfirst.frc3550.Julius2018.commands.*;
//import org.usfirst.frc3550.Robotronix2017.commands.Asp;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released  and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());


	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	private Joystick joystickPilote = new Joystick(0);
	private Joystick joystickCoPilot = new Joystick(1);

	public OI() {
		// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
		// SmartDashboard Buttons

		//Button button1 = new JoystickButton(joyRight, 1);
		//button1.whileHeld(new OpenPince());

		Button button1 = new JoystickButton(joystickCoPilot, 1);
		button1.whileHeld(new DropCubeCommand());

		Button button2 = new JoystickButton(joystickCoPilot, 2);
		button2.whileHeld(new PickupCubeCommand());

		//Button test = new JoystickButton(joyRight, 5);
		//test.whenPressed(new Speed2Command());

		Button button3 = new JoystickButton(joystickCoPilot, 3);
		button3.whileHeld(new GrimpeurUpCommand());

		Button button4 = new JoystickButton(joystickCoPilot, 4);
		button4.whileHeld(new GrimpeurDownCommand());

		Button button10 = new JoystickButton(joystickCoPilot, 10);
		button10.whileHeld(new GrandCadreDownCommand());

		Button button14 = new JoystickButton(joystickPilote, 6);
		button14.whenPressed(new Speed1Command());

		Button button15 = new JoystickButton(joystickPilote, 3);
		button15.whileHeld(new CheeringCommand());

		Button visionModeButton = new JoystickButton(joystickPilote, 4);
		visionModeButton.whileHeld(new VisionCamera());

		Button button5 = new JoystickButton(joystickPilote, 5);
		button5.whenPressed(new Speed2Command());

		Button button9 = new JoystickButton(joystickCoPilot, 9);//joyRight
		button9.whileHeld(new GrandCadreUpCommand());

		Button button8 = new JoystickButton(joystickCoPilot, 8);
		button8.whileHeld(new PetitCadreDownCommand());
		////Button button7 = new JoystickButton(joyRight, 7);
		////button7.whileHeld(new ElevatorDownCommand());

		Button button7= new JoystickButton(joystickCoPilot, 7);
		button7.whileHeld(new  PetitCadreUpCommand());
		
		Button button8P = new JoystickButton(joystickPilote, 8);
		button8P.whenPressed(new DriveStraightCommandTheory6());
		
		Button buttonRotate = new JoystickButton(joystickPilote, 9); //for tests purpose only, to be commented
		//buttonRotate.whenPressed(new AutoDriveDistanceCommand(50, 1));//
		//buttonRotate.whenPressed(new Theory6TurnToAngleCommand(90, 0.7, 0.25));
		//buttonRotate.whenPressed(new Theory6driveSraightCommand(50, 0.7, 90, 0.25));
		buttonRotate.whenPressed(new Theory6BasicCommandGroup());

		//Button button8 = new JoystickButton(joyRight, 8);
		//button8.whenPressed(new Pos1ColorLeftObjCubeBasculeCommand()); //Pos1ColorLeftObjCubeBalanceCommand());

		//Button button9 = new JoystickButton(joyRight, 9);
		//button9.whenPressed(new DriveRotateCommand(10,0));

		Button button12 = new JoystickButton(joystickCoPilot, 12);
		button12.whenPressed(new ClosePince());

		Button button11 = new JoystickButton(joystickCoPilot, 11);
		button11.whenPressed(new OpenPince());

		// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
	}

	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

	public Joystick getPilotStick() {
		return joystickPilote;
	}

	public double getRightStickL() {
		return joystickPilote.getY();
	}

	public double getRightStickR() {
		return joystickCoPilot.getY();
	}

	public double filterJoystickYAxis(double inputAxis){
		if ((inputAxis < -0.10)||(inputAxis > 0.1)){
			//if ((Math.abs(inputAxis) < 0.1)){
			return inputAxis;
		}
		else {
			return 0.0;
		}
	}
	public double filterJoystickXAxis(double inputAxis){
		if ((inputAxis < -0.3)||(inputAxis > 0.4)){  //{if ((inputAxis < -0.1)||(inputAxis > 0.1)){
			//if ((Math.abs(inputAxis) < 0.2)){
			return inputAxis;
		}
		else {
			return 0.0;
		}
	}
}
