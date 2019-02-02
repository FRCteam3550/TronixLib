/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package TronixLib;

//import java.awt.geom.Arc2D.Double;
import java.util.StringJoiner;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;


/**
 * A class for driving differential drive/skid-steer drive platforms such as the Kit of Parts drive
 * base, "tank drive", or West Coast Drive.
 *
 * <p>These drive bases typically have drop-center / skid-steer with two or more wheels per side
 * (e.g., 6WD or 8WD). This class takes a SpeedController per side. For four and
 * six motor drivetrains, construct and pass in {@link edu.wpi.first.wpilibj.SpeedControllerGroup}
 * instances as follows.
 *
 * <p>Four motor drivetrain:
 * <pre><code>
 * public class Robot {
 *   Spark m_frontLeft = new Spark(1);
 *   Spark m_rearLeft = new Spark(2);
 *   SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_rearLeft);
 *
 *   Spark m_frontRight = new Spark(3);
 *   Spark m_rearRight = new Spark(4);
 *   SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);
 *
 *   DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
 * }
 * </code></pre>
 *
 * <p>Six motor drivetrain:
 * <pre><code>
 * public class Robot {
 *   Spark m_frontLeft = new Spark(1);
 *   Spark m_midLeft = new Spark(2);
 *   Spark m_rearLeft = new Spark(3);
 *   SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_midLeft, m_rearLeft);
 *
 *   Spark m_frontRight = new Spark(4);
 *   Spark m_midRight = new Spark(5);
 *   Spark m_rearRight = new Spark(6);
 *   SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_midRight, m_rearRight);
 *
 *   DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
 * }
 * </code></pre>
 *
 * <p>A differential drive robot has left and right wheels separated by an arbitrary width.
 *
 * <p>Drive base diagram:
 * <pre>
 * |_______|
 * | |   | |
 *   |   |
 * |_|___|_|
 * |       |
 * </pre>
 *
 * <p>Each drive() function provides different inverse kinematic relations for a differential drive
 * robot. Motor outputs for the right side are negated, so motor direction inversion by the user is
 * usually unnecessary.
 *
 * <p>This library uses the NED axes convention (North-East-Down as external reference in the world
 * frame): http://www.nuclearprojects.com/ins/images/axis_big.png.
 *
 * <p>The positive X axis points ahead, the positive Y axis points right, and the positive Z axis
 * points down. Rotations follow the right-hand rule, so clockwise rotation around the Z axis is
 * positive.
 *
 * <p>Inputs smaller then {@value edu.wpi.first.wpilibj.drive.RobotDriveBase#kDefaultDeadband} will
 * be set to 0, and larger values will be scaled so that the full range is still used. This
 * deadband value can be changed with {@link #setDeadband}.
 *
 * <p>RobotDrive porting guide:
 * <br>{@link #tankDrive(double, double)} is equivalent to
 * {@link edu.wpi.first.wpilibj.RobotDrive#tankDrive(double, double)} if a deadband of 0 is used.
 * <br>{@link #arcadeDrive(double, double)} is equivalent to
 * {@link edu.wpi.first.wpilibj.RobotDrive#arcadeDrive(double, double)} if a deadband of 0 is used
 * and the the rotation input is inverted eg arcadeDrive(y, -rotation)
 * <br>{@link #curvatureDrive(double, double, boolean)} is similar in concept to
 * {@link edu.wpi.first.wpilibj.RobotDrive#drive(double, double)} with the addition of a quick turn
 * mode. However, it is not designed to give exactly the same response.
 */
public class DifferentialDriveTronix extends RobotDriveBase {
  public static final double kDefaultQuickStopThreshold = 0.2;
  public static final double kDefaultQuickStopAlpha = 0.1;

  private static int instances;

  private final SpeedController m_leftMotor;
  private final SpeedController m_rightMotor;

  private double m_quickStopThreshold = kDefaultQuickStopThreshold;
  private double m_quickStopAlpha = kDefaultQuickStopAlpha;
  private double m_quickStopAccumulator;
  private double m_rightSideInvertMultiplier = -1.0;
  private boolean m_reported;

  //private double m_currentForwardSpeed; 
  //private double m_currentTunringSpeed;

  private double m_leftMotorOutput;
  private double m_rightMotorOutput;


  /**
   * Construct a DifferentialDrive.
   *
   * <p>To pass multiple motors per side, use a {@link SpeedControllerGroup}. If a motor needs to be
   * inverted, do so before passing it in.
   */
  public DifferentialDriveTronix(SpeedController leftMotor, SpeedController rightMotor) {
    verify(leftMotor, rightMotor);
    //verify();
    m_leftMotor = leftMotor;
    m_rightMotor = rightMotor;
    //m_currentForwardSpeed = 0;
    //m_currentTunringSpeed = 0;
    addChild(m_leftMotor);
    addChild(m_rightMotor);
    instances++;
    setName("DifferentialDriveTronix", instances);
  }

  /**
   * Verifies that all motors are nonnull, throwing a NullPointerException if any of them are.
   * The exception's error message will specify all null motors, e.g. {@code
   * NullPointerException("leftMotor, rightMotor")}, to give as much information as possible to
   * the programmer.
   *
   * @throws NullPointerException if any of the given motors are null
   */
  //@SuppressWarnings("PMD.AvoidThrowingNullPointerException")
  private void verify(SpeedController leftMotor, SpeedController rightMotor) {
    if (leftMotor != null && rightMotor != null) {
      return;
    }
    StringJoiner joiner = new StringJoiner(", ");
    if (leftMotor == null) {
      joiner.add("leftMotor");
    }
    if (rightMotor == null) {
      joiner.add("rightMotor");
    }
    throw new NullPointerException(joiner.toString());
  }

  /**
   * Arcade drive method for differential drive platform.
   * The calculated values will be squared to decrease sensitivity at low speeds.
   *
   * @param xSpeed    The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *                  positive.
   */
  //@SuppressWarnings("ParameterName")
 // public void arcadeDrive(double xSpeed, double zRotation) {
  //  arcadeDrive(xSpeed, zRotation, forw);
 // }

  /**
   * Arcade drive method for differential drive platform.
   *
   * @param xSpeed        The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation     The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *                      positive.
   * @param squareInputs If set, decreases the input sensitivity at low speeds.
   */
  //@SuppressWarnings("ParameterName")
  public void arcadeDrive(double xSpeed, double zRotation, double currentForwardSpeed, double currentTurningSpeed) {
    if (!m_reported) {
      HAL.report(tResourceType.kResourceType_RobotDrive, 2,
                 tInstances.kRobotDrive2_DifferentialArcade);
      m_reported = true;
    }

    
    //m_currentForwardSpeed = currentForwardSpeed;
    //m_currentTunringSpeed = currentTurningSpeed;

    xSpeed = limit(xSpeed);
    //xSpeed = applyDeadband(xSpeed, m_deadband);

    zRotation = limit(zRotation);
    //zRotation = applyDeadband(zRotation, m_deadband);

    // Square the inputs (while preserving the sign) to increase fine control
    // while permitting full power.
  //  if (squareInputs) {
  //    xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
  //    zRotation = Math.copySign(zRotation * zRotation, zRotation);
   // }

   

    //double leftMotorOutput;
    //double rightMotorOutput;

    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

    if (xSpeed >= 0.0) {
      // First quadrant, else second quadrant
      if (zRotation >= 0.0) {
        m_leftMotorOutput = maxInput;
        m_rightMotorOutput = xSpeed - zRotation;
      } else {
        m_leftMotorOutput = xSpeed + zRotation;
        m_rightMotorOutput = maxInput;
      }
    } else {
      // Third quadrant, else fourth quadrant
      if (zRotation >= 0.0) {
        m_leftMotorOutput = xSpeed + zRotation;
        m_rightMotorOutput = maxInput;
      } else {
        m_leftMotorOutput = maxInput;
        m_rightMotorOutput = xSpeed - zRotation;
      }
    }

    boostInputs(currentForwardSpeed, currentTurningSpeed);
  
    m_leftMotor.set(limit(m_leftMotorOutput) * m_maxOutput);
    m_rightMotor.set(limit(m_rightMotorOutput) * m_maxOutput * m_rightSideInvertMultiplier );

    feed();
  }

  public void boostInputs(double currentForwardSpeed, double currentTurningSpeed){
    //m_currentForwardSpeed = 1.0;  // Pour eliminer une erreur de compilation inutile
    //m_currentTunringSpeed = 1.0;

    // permaBoost (testing purposes only)    
    if (m_leftMotorOutput != 0 && m_rightMotorOutput != 0){
    m_leftMotorOutput = m_leftMotorOutput + 0.3;
    m_rightMotorOutput = m_rightMotorOutput + 0.3;
   }
    /*  m_ some stuff
    //straight boosts
    if (currentTurningSpeed == 0 && leftMotorOutput == rightMotorOutput){
    //highBoost 
     if (currentForwardSpeed == 0 && leftMotorOutput >= 0.2 && rightMotorOutput >= 0.2){
        leftMotorOutput = leftMotorOutput + 0.3;
        rightMotorOutput = rightMotorOutput + 0.3;
      }
      //midBoost - unused (set variables)
      //else if(currentForwardSpeed == 0 && leftMotorOutput >= 0 && rightMotorOutput >= 0){
        //  leftMotorOutput = leftMotorOutput + 0.2;
         // rightMotorOutput = rightMotorOutput + 0.2;
      //}
      // lowBoost
      else if (currentForwardSpeed == 0  && leftMotorOutput >= 0.5 && rightMotorOutput >= 0.5){
          leftMotorOutput = leftMotorOutput + 0.1;
          rightMotorOutput = rightMotorOutput + 0.1;
      }
      //highMinusBoost - unused (set variables)
     // else if (currentForwardSpeed == 0 && leftMotorOutput >= 0 && rightMotorOutput >= 0) {
      //    leftMotorOutput = leftMotorOutput - 0.3;
       //   rightMotorOutput = rightMotorOutput - 0.3;
      //}
      //midMinusBoost - unused (set variables)
     // else if (currentForwardSpeed == 0 && leftMotorOutput >= 0 && rightMotorOutput >= 0) {
       //   leftMotorOutput = leftMotorOutput - 0.2;
        //  rightMotorOutput = rightMotorOutput - 0.2;
     // }
      // lowMinusBoost
      else if (currentForwardSpeed == 1.0 && leftMotorOutput >= 0.2 && rightMotorOutput >= 0.2) {
        leftMotorOutput = leftMotorOutput - 0.1;
        rightMotorOutput = rightMotorOutput - 0.1;
      }
      
    }
      // NOT YET USABLE - (wrong variables)
      // turn boosts - no forward speed
      else if (currentForwardSpeed == 0 && leftMotorOutput != rightMotorOutput) {
          //highBoost 
     if (currentTurningSpeed == 0 && leftMotorOutput >= 0.2 && rightMotorOutput >= 0.2){
        leftMotorOutput = leftMotorOutput + 0.3;
        rightMotorOutput = rightMotorOutput + 0.3;
      }
      //midBoost - unused (set variables)
      //else if(currentTurningSpeed == 0 && leftMotorOutput >= 0 && rightMotorOutput >= 0){
     //     leftMotorOutput = leftMotorOutput + 0.2;
     //     rightMotorOutput = rightMotorOutput + 0.2;
     // }
      // lowBoost
      else if (currentTurningSpeed == 0  && leftMotorOutput >= 0.5 && rightMotorOutput >= 0.5){
          leftMotorOutput = leftMotorOutput + 0.1;
          rightMotorOutput = rightMotorOutput + 0.1;
      }
      //highMinusBoost - unused (set variables)
     // else if (currentTunringSpeed == 0 && leftMotorOutput >= 0 && rightMotorOutput >= 0) {
       //   leftMotorOutput = leftMotorOutput - 0.3;
        //  rightMotorOutput = rightMotorOutput - 0.3;
      //}
      //midMinusBoost - unused (set variables)
     // else if (currentTurningSpeed == 0 && leftMotorOutput >= 0 && rightMotorOutput >= 0) {
      //    leftMotorOutput = leftMotorOutput - 0.2;
        //  rightMotorOutput = rightMotorOutput - 0.2;
     // }
      // lowMinusBoost
      else if (currentTurningSpeed == 1.0 && leftMotorOutput >= 0.2 && rightMotorOutput >= 0.2) {
        leftMotorOutput = leftMotorOutput - 0.1;
        rightMotorOutput = rightMotorOutput - 0.1;

      }
       // NOT YET USABLE - (wrong variables & names)
      // turn boosts w/forward speed 
      else if (currentForwardSpeed != 0 && leftMotorOutput != rightMotorOutput){
          //highBoost 
     if (currentForwardSpeed == 0 && leftMotorOutput >= 0.2 && rightMotorOutput >= 0.2){
        leftMotorOutput = leftMotorOutput + 0.3;
        rightMotorOutput = rightMotorOutput + 0.3;
      }
      //midBoost - unused (set variables)
      //else if(currentForwardSpeed == 0 && leftMotorOutput >= 0 && rightMotorOutput >= 0){
      //    leftMotorOutput = leftMotorOutput + 0.2;
      //    rightMotorOutput = rightMotorOutput + 0.2;
     // }
      // lowBoost
      else if (currentForwardSpeed == 0  && leftMotorOutput >= 0.5 && rightMotorOutput >= 0.5){
          leftMotorOutput = leftMotorOutput + 0.1;
          rightMotorOutput = rightMotorOutput + 0.1;
      }
      //highMinusBoost - unused (set variables)
     // else if (currentForwardSpeed == 0 && leftMotorOutput >= 0 && rightMotorOutput >= 0) {
     //     leftMotorOutput = leftMotorOutput - 0.3;
     //     rightMotorOutput = rightMotorOutput - 0.3;
     // }
      //midMinusBoost - unused (set variables)
     // else if (currentForwardSpeed == 0 && leftMotorOutput >= 0 && rightMotorOutput >= 0) {
     //     leftMotorOutput = leftMotorOutput - 0.2;
     //     rightMotorOutput = rightMotorOutput - 0.2;
     // }
      // lowMinusBoost
      else if (currentForwardSpeed == 1.0 && leftMotorOutput >= 0.2 && rightMotorOutput >= 0.2) {
        leftMotorOutput = leftMotorOutput - 0.1;
        rightMotorOutput = rightMotorOutput - 0.1;


      }
      }*/
    }

  /**
   * Curvature drive method for differential drive platform.
   *
   * <p>The rotation argument controls the curvature of the robot's path rather than its rate of
   * heading change. This makes the robot more controllable at high speeds. Also handles the
   * robot's quick turn functionality - "quick turn" overrides constant-curvature turning for
   * turn-in-place maneuvers.
   *
   * @param xSpeed      The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation   The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *                    positive.
   * @param isQuickTurn If set, overrides constant-curvature turning for
   *                    turn-in-place maneuvers.
   */
  //@SuppressWarnings({"ParameterName", "PMD.CyclomaticComplexity"}) 
  public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
    if (!m_reported) {
      HAL.report(tResourceType.kResourceType_RobotDrive, 2,
                 tInstances.kRobotDrive2_DifferentialCurvature);
      m_reported = true;
    }

    xSpeed = limit(xSpeed);
    xSpeed = applyDeadband(xSpeed, m_deadband);

    zRotation = limit(zRotation);
    zRotation = applyDeadband(zRotation, m_deadband);

    double angularPower;
    boolean overPower;

    if (isQuickTurn) {
      if (Math.abs(xSpeed) < m_quickStopThreshold) {
        m_quickStopAccumulator = (1 - m_quickStopAlpha) * m_quickStopAccumulator
            + m_quickStopAlpha * limit(zRotation) * 2;
      }
      overPower = true;
      angularPower = zRotation;
    } else {
      overPower = false;
      angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;

      if (m_quickStopAccumulator > 1) {
        m_quickStopAccumulator -= 1;
      } else if (m_quickStopAccumulator < -1) {
        m_quickStopAccumulator += 1;
      } else {
        m_quickStopAccumulator = 0.0;
      }
    }

    double leftMotorOutput = xSpeed + angularPower;
    double rightMotorOutput = xSpeed - angularPower;

    // If rotation is overpowered, reduce both outputs to within acceptable range
    if (overPower) {
      if (leftMotorOutput > 1.0) {
        rightMotorOutput -= leftMotorOutput - 1.0;
        leftMotorOutput = 1.0;
      } else if (rightMotorOutput > 1.0) {
        leftMotorOutput -= rightMotorOutput - 1.0;
        rightMotorOutput = 1.0;
      } else if (leftMotorOutput < -1.0) {
        rightMotorOutput -= leftMotorOutput + 1.0;
        leftMotorOutput = -1.0;
      } else if (rightMotorOutput < -1.0) {
        leftMotorOutput -= rightMotorOutput + 1.0;
        rightMotorOutput = -1.0;
      }
    }

    // Normalize the wheel speeds
    double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
    if (maxMagnitude > 1.0) {
      leftMotorOutput /= maxMagnitude;
      rightMotorOutput /= maxMagnitude;
    }

    m_leftMotor.set(leftMotorOutput * m_maxOutput);
    m_rightMotor.set(rightMotorOutput * m_maxOutput * m_rightSideInvertMultiplier);

    feed();
  }

  /**
   * Tank drive method for differential drive platform.
   * The calculated values will be squared to decrease sensitivity at low speeds.
   *
   * @param leftSpeed  The robot's left side speed along the X axis [-1.0..1.0]. Forward is
   *                   positive.
   * @param rightSpeed The robot's right side speed along the X axis [-1.0..1.0]. Forward is
   *                   positive.
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    tankDrive(leftSpeed, rightSpeed, true);
  }

  /**
   * Tank drive method for differential drive platform.
   *
   * @param leftSpeed     The robot left side's speed along the X axis [-1.0..1.0]. Forward is
   *                      positive.
   * @param rightSpeed    The robot right side's speed along the X axis [-1.0..1.0]. Forward is
   *                      positive.
   * @param squareInputs If set, decreases the input sensitivity at low speeds.
   */
  public void tankDrive(double leftSpeed, double rightSpeed, boolean squareInputs) {
    if (!m_reported) {
      HAL.report(tResourceType.kResourceType_RobotDrive, 2,
                 tInstances.kRobotDrive2_DifferentialTank);
      m_reported = true;
    }

    leftSpeed = limit(leftSpeed);
    leftSpeed = applyDeadband(leftSpeed, m_deadband);

    rightSpeed = limit(rightSpeed);
    rightSpeed = applyDeadband(rightSpeed, m_deadband);

    // Square the inputs (while preserving the sign) to increase fine control
    // while permitting full power.
    if (squareInputs) {
      leftSpeed = Math.copySign(leftSpeed * leftSpeed, leftSpeed);
      rightSpeed = Math.copySign(rightSpeed * rightSpeed, rightSpeed);
    }

    m_leftMotor.set(leftSpeed * m_maxOutput);
    m_rightMotor.set(rightSpeed * m_maxOutput * m_rightSideInvertMultiplier);

    feed();
  }

  /**
   * Sets the QuickStop speed threshold in curvature drive.
   *
   * <p>QuickStop compensates for the robot's moment of inertia when stopping after a QuickTurn.
   *
   * <p>While QuickTurn is enabled, the QuickStop accumulator takes on the rotation rate value
   * outputted by the low-pass filter when the robot's speed along the X axis is below the
   * threshold. When QuickTurn is disabled, the accumulator's value is applied against the computed
   * angular power request to slow the robot's rotation.
   *
   * @param threshold X speed below which quick stop accumulator will receive rotation rate values
   *                  [0..1.0].
   */
  public void setQuickStopThreshold(double threshold) {
    m_quickStopThreshold = threshold;
  }

  /**
   * Sets the low-pass filter gain for QuickStop in curvature drive.
   *
   * <p>The low-pass filter filters incoming rotation rate commands to smooth out high frequency
   * changes.
   *
   * @param alpha Low-pass filter gain [0.0..2.0]. Smaller values result in slower output changes.
   *              Values between 1.0 and 2.0 result in output oscillation. Values below 0.0 and
   *              above 2.0 are unstable.
   */
  public void setQuickStopAlpha(double alpha) {
    m_quickStopAlpha = alpha;
  }

  /**
   * Gets if the power sent to the right side of the drivetrain is multipled by -1.
   *
   * @return true if the right side is inverted
   */
  public boolean isRightSideInverted() {
    return m_rightSideInvertMultiplier == -1.0;
  }

  /**
   * Sets if the power sent to the right side of the drivetrain should be multipled by -1.
   *
   * @param rightSideInverted true if right side power should be multipled by -1
   */
  public void setRightSideInverted(boolean rightSideInverted) {
    m_rightSideInvertMultiplier = rightSideInverted ? -1.0 : 1.0;
  }

  @Override
  public void stopMotor() {
    m_leftMotor.stopMotor();
    m_rightMotor.stopMotor();
    feed();
  }

  @Override
  public String getDescription() {
    return "DifferentialDrive";
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("DifferentialDrive");
    builder.setActuator(true);
    builder.setSafeState(this::stopMotor);
    builder.addDoubleProperty("Left Motor Speed", m_leftMotor::get, m_leftMotor::set);
    builder.addDoubleProperty(
        "Right Motor Speed",
        () -> m_rightMotor.get() * m_rightSideInvertMultiplier,
        x -> m_rightMotor.set(x * m_rightSideInvertMultiplier));
  }
}
