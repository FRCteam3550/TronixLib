package org.usfirst.frc3550.Julius2018.theory6.pathing;

/**
 * This class is used to save constant values that will be changed every once in awhile. This can include PID tuned
 * values or autonomous distances, etc...
 *
 * @author Mahrus Kazi
 * @author Bryan Kristiono
 * @since 2015-09-13
 */

//Competition
public class NumberConstants {
	//**************************************************************************
    //*************************** PID VALUES (DRIVE) ***************************
    //**************************************************************************
	
	//Competition
	public static final double pDrive 									 = 0.08; //0.05
	public static final double iDrive 									 = 0.00;
	public static final double dDrive 									 = 0.01; //0.008
	
	public static final double Drive_Scale 								 = 0.6; //0.6
	
	//**************************************************************************
    //**************************** PID VALUES (GYRO) ***************************
    //**************************************************************************
	
	//Competition
	public static final double pGyro 									 = 0.02; //0.0125
	public static final double iGyro 									 = 0.00;
	public static final double dGyro 									 = 0.00;
	
	//**************************************************************************
    //*************************** PID VALUES (TURRET) **************************
    //**************************************************************************
	
	//Competition
	public static final double pTurret 									 = 0.025;
	public static final double iTurret 									 = 0.00;
	public static final double dTurret 									 = 0.005;
	
	//**************************************************************************
    //*************************** PID VALUES (CAMERA) **************************
    //**************************************************************************
	
	//Competition
	public static final double pCamera 									 = 0.07;
	public static final double iCamera 									 = 0.00;
	public static final double dCamera 									 = 0.000;
	
	public static final double cameraOffset								= 2.5;
	
	//**************************************************************************
    //************************** PID VALUES (SHOOTER) **************************
    //**************************************************************************
	
	//Competition
	public static final double pShooterBadder							= 0.00023;
	public static final double iShooterBadder							= 0.0000;
	public static final double dShooterBadder							= 0.0000;
	public static final double kForward									= 0.000137500;
	public static final double bForward									= 0.080000000;
//	public static final double kForward									= 0.000157668;
//	public static final double bForward									= 0.001564417;
	
	//**************************************************************************
    //**************************** PID VALUES (ARM) ****************************
    //**************************************************************************
	
	//Competition
	public static final double pArm 									 = 0.006;
	public static final double iArm 									 = 0.00;
	public static final double dArm 									 = 0.00;
	
	//**************************************************************************
    //**************************OUTPUT VALUES (Shooter)*************************
    //**************************************************************************
	
	//Competition
	public static final int downArmAngle								= 690;
	//Arm all the way up
	public static final int upArmAngle									= 970;
	
	
	//**************************************************************************
	//***************************** SHOOTER NUMBERS ****************************
	//**************************************************************************
	
	//Competition
	public static final int outerShotRPM								= 4600;
	public static final int spyShotRPM									= 4200;
	public static final int badderShotRPM								= 4400;
	
	public static final int spyShotAngle								= -78;
	public static final int backAngle									= -180;
	
	public static final double waitForPop								= 0.75;
	public static final double waitForHolder							= 0.15;
	public static final double waitForHolderClose						= 0.50;	

	//**************************************************************************
	//***************************** INTAKE NUMBERS *****************************
	//**************************************************************************
	
	public static final double intakeDist								= 12.2;
	
	//**************************************************************************
	//****************************** AUTO NUMBERS ******************************
	//**************************************************************************

	//END LOCATION
	public static final int DEFAULT											= 0;
	public static final int LEFT											= 1;
	public static final int CENTER											= 2;
	public static final int RIGHT											= 3;

	//DEFENSES
	public static final int PORTCULLIS										= 0;
	public static final int CHEVAL											= 1;
	public static final int SALLYPORT										= 2;
	public static final int DRAWBRIDGE										= 3;
	public static final int ROCKWALL										= 4;
	public static final int ROUGHTERRAIN									= 5;
	public static final int MOAT											= 6;
}
