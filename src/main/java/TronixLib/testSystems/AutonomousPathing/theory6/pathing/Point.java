package org.usfirst.frc3550.Julius2018.theory6.pathing;

/**
 * A Point class that allows the input of doubles.
 * 
 * @author Mahrus Kazi
 */
public class Point {

	private double x;
	private double y;

	/**
	 * Constructor that creates a Point object.
	 * 
	 * @param x
	 *            x coordinate
	 * @param y
	 *            y coordinate
	 */
	public Point(double x, double y) {
		this.x = x;
		this.y = y;
	}

	/**
	 * @return The x coordinate
	 */
	public double getX() {
		return this.x;
	}

	/**
	 * @return The y coordinate
	 */
	public double getY() {
		return this.y;
	}
}
