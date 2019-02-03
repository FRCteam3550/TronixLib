package org.usfirst.frc3550.Julius2018.commands.pathing;

import java.util.function.Function;

public class SpeedGenerator {
	private double minSpeed, maxSpeed, startAccel, stopAccel, radius1, radius2, lookahead;
	
	/**
	 * Automatically generate a speed function
	 * @param minSpeed minimum speed to run during any given path
	 * @param maxSpeed maximum speed to run during any given path
	 * @param accel start/stop acceleration, in percent (0-1) per inch
	 * @param radius1 turn radius to start slowing down, in inches
	 * @param radius2 turn radius to run at minimum speed, in inches
	 * @param lookahead look ahead for curves, in inches
	 */
	public SpeedGenerator(double minSpeed, double maxSpeed, double startAccel, double stopAccel, double radius1, double radius2, double lookahead) {
		this.minSpeed = minSpeed;
		this.maxSpeed = maxSpeed;
		this.startAccel = startAccel;
		this.stopAccel = stopAccel;
		this.radius1 = radius1;
		this.radius2 = radius2;
		this.lookahead = lookahead;
	}
	
	public Function<Double, Double> getSpeedFunction(Path path) {
		final double length = path.getTotalLength();
		return t-> {
			final double inchesTraveled = t * length;
			final double inchesToGo = (1 - t) * length;
			
			final double speedUpSpeed = minSpeed + inchesTraveled * startAccel;
			final double slowDownSpeed = minSpeed + inchesToGo * stopAccel;
			
			double turnSpeed = maxSpeed;
			double lookaheadt = t + lookahead/length;
			if(lookaheadt > 1.0) {
				lookaheadt = 1.0;
			}
			
			PathSegment segment = path.getPathSegmentAtDistance(lookaheadt * length);
			final double curvature = segment.getCurvature().apply((lookaheadt * length - path.getTotalOfCompletedPaths(lookaheadt * length))/segment.getLength());
			if(1 / curvature <= radius1) {
				turnSpeed = maxSpeed - (curvature - 1/radius1)/(1/radius2 - 1/radius1) * (maxSpeed - minSpeed);
			}
			
			double outSpeed = Math.min(Math.min(speedUpSpeed, slowDownSpeed), Math.min(turnSpeed, maxSpeed));
			if(outSpeed < minSpeed) {
				outSpeed = minSpeed;
			}
			
			return outSpeed;
		};
	}
	
	// these methods are provided primarily in case you want to construct
	// a second SpeedGenerator that has options similar to an original
	public double getMinSpeed() {
		return minSpeed;
	}
	public double getMaxSpeed() {
		return maxSpeed;
	}
	public double getStartAccel() {
		return startAccel;
	}
	public double getStopAccel() {
		return stopAccel;
	}
	public double getRadius1() {
		return radius1;
	}
	public double getRadius2() {
		return radius2;
	}
	public double getLookahead() {
		return lookahead;
	}
}
