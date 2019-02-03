package org.usfirst.frc3550.Julius2018.commands;
//package org.usfirst.frc3550.Julius2018.commands;
//
//import org.usfirst.frc3550.Julius2018.Robot;
//
//import edu.wpi.first.wpilibj.command.Command;
//
///**
// *
// */
//public class DriveDistanceCommand extends Command {
//
//	int m_Distance;
//	int m_Direction;
//	int m_v0;
//	int m_v3;
//	int m_CurDistance;
//	int m_CurIteration;
//	double m_CurSpeed;
//	double m_VMax;
//	double m_A;
//	double m_t1;
//	double m_t2;
//	double m_t3;
//	double m_tplateau;
//	double m_t4;
//	double m_d1;
//	double m_d2;
//	double m_d3;
//	double m_d4;
//	
//	
//	// Distance in mm
//	// Direction = 0: Backward 1:Forward
//    public DriveDistanceCommand(int distance, int direction)
//    {
//    	// drivebase subsystem
//        requires(Robot.driveTrain);
//        
//        m_Distance  = distance;
//        m_Direction = direction;
//        
//        m_VMax = 24;        // 24" per second
//        m_A = m_VMax / 1.0; // Rejoindre vitesse max en 1 sec
//    }
//
//    // Called just before this Command runs the first time
//    protected void initialize() {
//    	//System.out.println("Running DriveDistanceCommand for "+m_Distance+"mm");
//    	
//    	Robot.driveTrain.SetDistance(m_Distance, m_VMax, m_A);
//    }
//
//    public void SetDistance(int distance, double vmax, double a)
//	{
//		// On assume v0 = v3 = 0 pour l'instant
//		m_v0 = 0;
//		m_v3 = 0;
//		m_Distance = distance;
//		m_CurDistance = 0;
//		m_CurIteration = 0;
//		m_VMax = vmax;
//		m_A = a;
//
//		m_t1 = (m_VMax-m_v0) / m_A; // time from v0 to vmax (time to reach full speed)
//        m_t4 = (m_VMax-m_v3) / m_A;   // time from vmax to v3 (time to brake)
//        m_d1 = m_v0*m_t1 + 0.5*m_A*m_t1*m_t1; // distance t0-t1
//        m_d2 = m_v3*m_t4 + 0.5*m_A*m_t4*m_t4; // distance t2-t3
//
//		if (m_d1+m_d2 < m_Distance )
//		{
//			// plateau at vmax in the middle
//			m_tplateau = ( m_Distance - m_d1 - m_d2 ) / m_VMax;
//			m_t2 = m_t1 + m_tplateau;
//			m_t3 = m_t2 + m_t4;
//		}
//		else
//		{
//			double brake_distance = 10.0;
//			
//			// start breaking before reaching vmax
//			// http://wikipedia.org/wiki/Classical_mechanics#1-Dimensional_Kinematics
//			m_t1 = ( Math.sqrt( 2.0*m_A*brake_distance + m_v0*m_v0 ) - m_v0 ) / m_A;
//			m_t2 = m_t1;
//			m_t3 = m_t2 + ( Math.sqrt( 2.0*m_A*(m_Distance-brake_distance) + m_v3*m_v3 ) - m_v3 ) / m_A;
//		}
//	}
//	
//    // Called repeatedly when this Command is scheduled to run
//    protected void execute() {
//    	Robot.driveTrain.DistanceMove();
//    }
//
//	public double DistanceMove(){
//		return m_CurSpeed;
//	}
//	
//    // Make this return true when this Command no longer needs to run execute()
//    protected boolean isFinished() {
//        return Robot.driveTrain.isDistanceDone();
//    }
//
//    // Called once after isFinished returns true
//    protected void end() {
//    	Robot.driveTrain.stop();
//    }
//
//    // Called when another command which requires one or more of the same
//    // subsystems is scheduled to run
//    protected void interrupted() {
//    	end();
//    }
//}
