package TronixLib;

/*import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.commands.GrabberPrendreCmd;
//import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
//import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 
public class ArcadeDrive extends Command {
    private Subsystem DriveBaseSub;
  public ArcadeDrive(Subsystem DriveBaseSubsystem) {
    // Use requires() here to declare subsystem dependencies
    DriveBaseSub = DriveBaseSubsystem;
    requires(DriveBaseSub);
  }

  // Called just before this Command runs the first time
  //@Override
  protected void initialize() {
    Robot.m_DriveBaseSub.stopRobot();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_DriveBaseSub.arcadeDrive(Robot.m_oi.returnPilote().getX(),Robot.m_oi.returnPilote().getY());
    /*if (squareInputs) {
      xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
      zRotation = Math.copySign(zRotation * zRotation, zRotation);
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_DriveBaseSub.stopRobot();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }*/
//}