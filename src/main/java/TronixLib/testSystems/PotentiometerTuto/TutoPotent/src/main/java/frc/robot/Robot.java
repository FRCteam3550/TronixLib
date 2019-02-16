/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.*;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.wpilibj.Compressor;
import com.ctre.phoenix.sensors.*;
//import com.ctre.phoenix.sensors.PigeonIMU.GeneralStatus;


/**
 * This is a sample program to demonstrate how to use a soft potentiometer and a
 * PID controller to reach and maintain position setpoints on an elevator
 * mechanism.
 */
public class Robot extends TimedRobot {
  private static final int kPotChannel = 1; //analog input pin
  private static final int kMotorChannel = 4; //CAN channel
  private static final int kJoystickChannel = 0; //USB number in DriverStation
 // private static final int buttonNumber = 4; // Joystick button
  private static final int kPDpanelChannel = 0; //power distribution panel channel
  private static  DriverStation driverStation; //power distribution panel channel
  private static DigitalInput photos0;
  private static DigitalInput photos1;
  private static DigitalInput photos2;
  //private static Compressor compressor;
 // public static PigeonIMU _pidgey; 
  // bottom, middle, and top elevator setpoints
  //private static final double[] kSetPoints = {1.0, 2.6, 4.3}; // bottom, middle, and top elevator setpoints

  // proportional, integral, and derivative speed constants; motor inverted
  // DANGER: when tuning PID constants, high/inappropriate values for kP, kI,
  // and kD may cause dangerous, uncontrollable, or undesired behavior!
  // these may need to be positive for a non-inverted motor
  //private static final double kP = -5.0; //proportional speed constant
  //private static final double kI = -0.02;
 // private static final double kD = -2.0;

    /* String for output */
  StringBuilder _sb = new StringBuilder();
    
    /* Loop tracker for prints */
  int _loops = 0;
    
    /** Track button state for single press event */
	boolean _lastButton1 = false;

	/** Save the target position to servo to */
  double targetPositionRotations1;
  double targetPositionRotations2;
  double targetPositionRotations3;

  //private PIDController m_pidController;
 // @SuppressWarnings("PMD.SingularField")
  private AnalogInput m_potentiometer;
 // @SuppressWarnings("PMD.SingularField")
  private WPI_TalonSRX m_elevatorMotor;
  private Joystick m_joystick;
  private PowerDistributionPanel m_ppd;
  private AHRS m_ahrs;

  /** A couple latched values to detect on-press events for buttons */
	boolean[] _previousBtns = new boolean[Constants.kNumButtonsPlusOne]; //testing LimitSwitch
  boolean[] _currentBtns = new boolean[Constants.kNumButtonsPlusOne]; //testing LimitSwitch
  


 // private int m_index;
 // private boolean m_previousButtonValue;

  @Override
  public void robotInit() {

    m_potentiometer = new AnalogInput(kPotChannel);
    m_joystick = new Joystick(kJoystickChannel);
    m_ppd = new PowerDistributionPanel(kPDpanelChannel);
    photos0 = new DigitalInput(0);
    photos1 = new DigitalInput(1);
    photos2 = new DigitalInput(2);
   // compressor = new Compressor(0);
    //_pidgey = new PigeonIMU(4);
   // compressor.setClosedLoopControl(true);
    


    try {
			/* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
			/* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
			/* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      m_ahrs = new AHRS(SPI.Port.kMXP); 
      //ahrs = new AHRS(SerialPort.Port.kUSB1); 
      
		} catch (RuntimeException ex ) {
			DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
		} 

    driverStation = DriverStation.getInstance();
    
    m_elevatorMotor = new WPI_TalonSRX(kMotorChannel);
    m_elevatorMotor.set(ControlMode.Position, 0);
    m_elevatorMotor.configFactoryDefault();
    /* Config sensor used for Primary PID [Velocity] */
   // m_elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
    /*m_elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog,
    Constants.kPIDLoopIdx, 
    Constants.kTimeoutMs); */
    m_elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,
    Constants.kPIDLoopIdx, 
    Constants.kTimeoutMs);
     /**
		 * Phase sensor accordingly. 
         * Positive Sensor Reading should match Green (blinking) Leds on Talon
         */
    m_elevatorMotor.setSensorPhase(true);
    /* Config the peak and nominal outputs, 12V means full */
		m_elevatorMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
		m_elevatorMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
		m_elevatorMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
		m_elevatorMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		m_elevatorMotor.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
		m_elevatorMotor.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		m_elevatorMotor.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		m_elevatorMotor.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
    m_elevatorMotor.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);
    
    /* Config Position Closed Loop gains in slot1, tsypically kF stays zero. */
	//	m_elevatorMotor.config_kF(Constants.kPIDLoopIdx1, Constants.kGains.kF, Constants.kTimeoutMs);
		//m_elevatorMotor.config_kP(Constants.kPIDLoopIdx1, Constants.kGains.kP, Constants.kTimeoutMs);
	//	m_elevatorMotor.config_kI(Constants.kPIDLoopIdx1, Constants.kGains.kI, Constants.kTimeoutMs);
		//m_elevatorMotor.config_kD(Constants.kPIDLoopIdx1, Constants.kGains.kD, Constants.kTimeoutMs);

/**
		 * Grab the 360 degree position of the MagEncoder's absolute
		 * position, and intitally set the relative sensor to match.
		 */
    /* for potentiometer
    int absolutePosition = m_elevatorMotor.getSensorCollection().getAnalogIn(); */
    int absolutePosition = m_elevatorMotor.getSensorCollection().getPulseWidthPosition();
    /* Set the quadrature (relative) sensor to match absolute */
    /*PID on Slot0                                             */
    m_elevatorMotor.setSelectedSensorPosition(absolutePosition, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    
    /*PID on Slot1                                             */
	  //	m_elevatorMotor.setSelectedSensorPosition(absolutePosition, Constants.kPIDLoopIdx1, Constants.kTimeoutMs);

    //m_pidController = new PIDController(kP, kI, kD, m_potentiometer, m_elevatorMotor);
   // m_pidController.setInputRange(0, 5);
  }

  @Override
  public void teleopInit() {
   // robotInit();
   m_ahrs.reset();
   
  }
 
  @Override
 public void robotPeriodic() {
  // super.robotPeriodic();
  //m_ahrs.reset();
  
 }

 @Override
  public void disabledInit() {
    //super.disabledInit();
  }

  @Override
  public void teleopPeriodic() {
/* Get gamepad axis */
//double leftYstick = -1 * m_joystick.getY();
//PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
   // PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
   // double [] xyz_dps = new double [4];
    //double _targetAngle = 0;
    AHRS.BoardYawAxis yaw_axis = m_ahrs.getBoardYawAxis();
    SmartDashboard.putString("YawAxisDirection", yaw_axis.up ? "Up" : "Down" );
    SmartDashboard.putNumber("YawAxis", yaw_axis.board_axis.getValue() );
          
 /* Sensor Board Information                                                 */
    SmartDashboard.putString("FirmwareVersion", m_ahrs.getFirmwareVersion());
    SmartDashboard.putNumber("Angle : ", m_ahrs.getYaw());
    SmartDashboard.putBoolean("ForwardLimitSwitchClosed", m_elevatorMotor.getSensorCollection().isFwdLimitSwitchClosed());
    SmartDashboard.putBoolean("ReverseLimitSwitchClosed", m_elevatorMotor.getSensorCollection().isRevLimitSwitchClosed());
    
    //SmartDashboard.putNumber("Potentiometer", m_elevatorMotor.getSensorCollection().getAnalogInRaw());
    SmartDashboard.putNumber("Encoder", m_elevatorMotor.getSensorCollection().getPulseWidthPosition());
    //SmartDashboard.putNumber("PigeonStatus" , _pidgey.getFusedHeading()); 
   // SmartDashboard.putNumber("FusionHeading", fusionStatus.heading );
    commonLoop();
    
		//_pidgey.getRawGyro(xyz_dps);
		//_pidgey.getFusedHeading(fusionStatus);
    //fusionStatus.heading
 }
   
     // CommonLoop(); testing limitSwitches


    // when the button is pressed once, the selected elevator setpoint
    // is incremented
    //if(m_joystick.getRawButton(1)){
   
   // SmartDashboard.putBoolean("isOperator", (driverStation.isOperatorControl()&driverStation.isEnabled()));
   //SmartDashboard.putNumber("Pot Position", m_elevatorMotor.getSelectedSensorVelocity());
   //SmartDashboard.putNumber("Pot Position", m_elevatorMotor.getSelectedSensorPosition());
    //};
   // if (currentButtonValue && !m_previousButtonValue) {
      // index of the elevator setpoint wraps around.
    //  m_index = (m_index + 1) % kSetPoints.length;
    //}
    //m_previousButtonValue = currentButtonValue;

    //m_pidController.setSetpoint(kSetPoints[m_index]);
  //}
  @Override
  public void disabledPeriodic(){

  }
  void commonLoop() {
		/* Gamepad processing */
		double leftYstick = m_joystick.getY();
		boolean button1 = m_joystick.getRawButton(1);	// X-Button
    boolean button2 = m_joystick.getRawButton(4);	// A-Button
    boolean button3 = m_joystick.getRawButton(5);	// A-Button
		/* Get Talon/Victor's current output percentage */
		double motorOutput = m_elevatorMotor.getSensorCollection().getPulseWidthPosition();

		/* Deadband gamepad */
		if (Math.abs(leftYstick) < 0.10) {
			/* Within 10% of zero */
			leftYstick = 0;
		}

		/* Prepare line to print */
		_sb.append("\tout:");
		/* Cast to int to remove decimal places */
		_sb.append((int) (motorOutput * 100));
		_sb.append("%");	// Percent
		_sb.append("\tpos:");
		_sb.append(m_elevatorMotor.getSensorCollection().getPulseWidthPosition());
		_sb.append("u"); 	// Native units

		/**
		 * When button 1 is pressed, perform Position Closed Loop to selected position,
		 * indicated by Joystick position x10, [-10, 10] rotations
		 */
		if (button1) {
			/* Position Closed Loop */
			/* 10 Rotations * 4096 u/rev in either direction */
      //targetPositionRotations = leftYstick * 10.0 * 4096;
      targetPositionRotations1 = 1 * 3 * 1000;
      SmartDashboard.putNumber("targetPositon", targetPositionRotations1);
      //SmartDashboard.putNumber("PotentiometerPosition", m_elevatorMotor.getSensorCollection().getAnalogIn());
      SmartDashboard.putNumber("PotentiometerPosition", m_elevatorMotor.getSensorCollection().getPulseWidthPosition());
			m_elevatorMotor.set(ControlMode.PercentOutput, targetPositionRotations1);
		}else if (button2) {/* When button 2 is held, just straight drive */
      /* Percent Output */
      targetPositionRotations2 = targetPositionRotations1 +1 * 3 * 100;
      SmartDashboard.putNumber("targetPositon", targetPositionRotations2);
      //SmartDashboard.putNumber("PotentiometerPosition", m_elevatorMotor.getSensorCollection().getAnalogIn());
      SmartDashboard.putNumber("PotentiometerPosition", m_elevatorMotor.getSensorCollection().getPulseWidthPosition());
      m_elevatorMotor.set(ControlMode.PercentOutput, targetPositionRotations2);
			//m_elevatorMotor.set(ControlMode.PercentOutput, leftYstick);
    }else if (button3) {
      /* Percent Output */
      targetPositionRotations3 = targetPositionRotations2 +1 * 3 * 100;
      SmartDashboard.putNumber("targetPositon", targetPositionRotations3);
      //SmartDashboard.putNumber("PotentiometerPosition", m_elevatorMotor.getSensorCollection().getAnalogIn());
      SmartDashboard.putNumber("PotentiometerPosition", m_elevatorMotor.getSensorCollection().getPulseWidthPosition());
      m_elevatorMotor.set(ControlMode.Position, targetPositionRotations3);
			//m_elevatorMotor.set(ControlMode.PercentOutput, leftYstick);
    }
    else{
      m_elevatorMotor.set(ControlMode.PercentOutput, 0);
    }

		/* If Talon is in position closed-loop, print some more info */
		if (m_elevatorMotor.getControlMode() == ControlMode.Position) {
			/* ppend more signals to print when in speed mode. */
			_sb.append("\terr:");
			_sb.append(m_elevatorMotor.getClosedLoopError(0));
			_sb.append("u");	// Native Units
			_sb.append("\ttrg:");
			_sb.append(targetPositionRotations1);
			_sb.append("u");	/// Native Units
		}

		/**
		 * Print every ten loops, printing too much too fast is generally bad
		 * for performance.
		 */
		if (++_loops >= 10) {
			_loops = 0;
			System.out.println(_sb.toString());
		}

		/* Reset built string for next loop */
		_sb.setLength(0);
		
		/* Save button state for on press detect */
		_lastButton1 = button1;
    }
  void SelectLimitSwitch(int choice) {
		if (choice == 0) {
			/* use feedback connector but disable feature, use-webdash to reenable */
			m_elevatorMotor.configForwardLimitSwitchSource(	LimitSwitchSource.FeedbackConnector,
													        LimitSwitchNormal.Disabled,
													        Constants.kTimeoutMs);

			m_elevatorMotor.configReverseLimitSwitchSource(	LimitSwitchSource.FeedbackConnector,
													        LimitSwitchNormal.Disabled,
													        Constants.kTimeoutMs);

			System.out.println("Limit Switches disabled.");
		} else if (choice == 1) {
			/* use feedback connector - use three functions */
			m_elevatorMotor.configForwardLimitSwitchSource(	LimitSwitchSource.FeedbackConnector,
													        LimitSwitchNormal.NormallyOpen,
													        Constants.kTimeoutMs);

			m_elevatorMotor.configReverseLimitSwitchSource(	LimitSwitchSource.FeedbackConnector,
													        LimitSwitchNormal.NormallyOpen,
													        Constants.kTimeoutMs);

			System.out.println("Limit Switches locally enabled.");
		} 
  }
  
  void getButtons(boolean[] _currentBtns) {
    for (int i = 1; i < Constants.kNumButtonsPlusOne; ++i) {
      _currentBtns[i] = m_joystick.getRawButton(i);
    }
  }

  double deadband(double value) {
    if (value >= +0.05) {
      return value;
    }
    if (value <= -0.05) {
      return value;
    }
    return 0;
  }

  public void configPotentiometerasSensor(){
    SmartDashboard.putBoolean("isOperator", (driverStation.isOperatorControl()&driverStation.isEnabled()));
    m_elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
    m_elevatorMotor.setSelectedSensorPosition(0); 
  }

  public void operateWishPotentiometer(){
    m_elevatorMotor.set(m_joystick.getY());
    SmartDashboard.putNumber("Pot Position", m_elevatorMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Pot Status", m_elevatorMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("PCM total current", m_ppd.getTotalCurrent());
    SmartDashboard.putNumber("Voltage Channel 13", m_ppd.getCurrent(13));
    SmartDashboard.putNumber("Voltage Channel 14", m_ppd.getCurrent(14));
    SmartDashboard.putNumber("Voltage Channel 15", m_ppd.getCurrent(15));
    SmartDashboard.putNumber("Voltage Channel 7", m_ppd.getCurrent(7));
    SmartDashboard.putBoolean("isAutonomous", driverStation.isTest());
    SmartDashboard.putBoolean("photoswitch0", photos0.get());
    SmartDashboard.putBoolean("photoswitch1", photos1.get());
    SmartDashboard.putBoolean("photoswitch2", photos2.get());
  }	
}
