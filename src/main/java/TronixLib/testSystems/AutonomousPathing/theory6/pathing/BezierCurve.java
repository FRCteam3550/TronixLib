package org.usfirst.frc3550.Julius2018.theory6.pathing;

import java.util.List;
import java.io.BufferedOutputStream;
import java.io.FileOutputStream;
import java.io.FilterOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;

/**
 * Class that generates a Bezier Curve based of off 4 coordinates given by the
 * client
 * 
 * Reference Material: https://www.desmos.com/calculator/cahqdxeshd
 * 
 * @author Mahrus Kazi
 * @version 2.0, 25 Sep 2016
 */

public class BezierCurve {

	/** Used to save number of points generated */
	private static int size = 2000; //change from 1000 to 100 for test purposes

	/** Used to store inputed 4 Bezier curves points coordinates */
	public Point[] vector = new Point[4];

	/** Stores all x coordinates points on the curve */
	public List<Double> xPoints = new ArrayList<Double>();

	/** Stores all y coordinates points on the curve */
	public List<Double> yPoints = new ArrayList<Double>();

	/** Stores distanced between coordinates */
	public List<Double> hypotenuse = new ArrayList<Double>();

	/** Used to store difference between x and x+1 */
	public List<Double> xDelta = new ArrayList<Double>();

	/** Used to store difference between y and y+1 */
	public List<Double> yDelta = new ArrayList<Double>();

	/** Used to store angles between two poins coordinates */
	public List<Double> angle = new ArrayList<Double>();

	/** Stores x coordinates values from inputed coordinates */
	public double[] xValues = new double[4];

	/** Stores y coordinates values from inputed coordinates */
	public double[] yValues = new double[4];

	/** Used to store total arc length */
	public double distance;

	/**
	 * Requires 4 points to generate Bezier Curves, inputed as Points.
	 *
	 * @param startPoint
	 *            the start point
	 * @param controlPoint1
	 *            the control point 1
	 * @param controlPoint2
	 *            the control point 2
	 * @param endPoint
	 *            the end point
	 */
	public BezierCurve(Point startPoint, Point controlPoint1, Point controlPoint2, Point endPoint) {
		vector[0] = startPoint;
		vector[1] = controlPoint1;
		vector[2] = controlPoint2;
		vector[3] = endPoint;
		putPoints();
		findPoints();
		calcPoints();
	}

	/**
	 * Instantiates a new bezier curve.
	 *
	 * @param startPoint
	 *            the start point
	 * @param controlPoint1
	 *            the control point 1
	 * @param controlPoint2
	 *            the control point 2
	 * @param endPoint
	 *            the end point
	 * @param size
	 *            the number points to be generated
	 */
	public BezierCurve(Point startPoint, Point controlPoint1, Point controlPoint2, Point endPoint, int size) {
		vector[0] = startPoint;
		vector[1] = controlPoint1;
		vector[2] = controlPoint2;
		vector[3] = endPoint;
		this.size = size;
		putPoints();
		findPoints();
		calcPoints();
	}

	/**
	 * Update given coordinates.
	 *
	 * @param startPoint
	 *            the start point
	 * @param controlPoint1
	 *            the control point 1
	 * @param controlPoint2
	 *            the control point 2
	 * @param endPoint
	 *            the end point
	 */
	public void changePoints(Point startPoint, Point controlPoint1, Point controlPoint2, Point endPoint) {
		vector[0] = startPoint;
		vector[1] = controlPoint1;
		vector[2] = controlPoint2;
		vector[3] = endPoint;
		putPoints();
		findPoints();
		calcPoints();
	}

	/**
	 * Generates the points on the Bezier Curve. p(t)=(x(t),y(t))
	 *
	 * @return Returns a ArrayList consisting of Points
	 */
	public ArrayList<Point> findPoints() {
		ArrayList<Point> points = new ArrayList<Point>();
		double xVal;
		double yVal;
		xPoints.clear();
		yPoints.clear();

		for (double counter = 0; counter <= size; counter++) { //parametric variable
			double t = counter / size;
			xVal = useFunctionX(xValues[0], xValues[1], xValues[2], xValues[3], t);
			yVal = useFunctionY(yValues[0], yValues[1], yValues[2], yValues[3], t);
			xPoints.add(xVal);
			yPoints.add(yVal);
			points.add(new Point(xVal, yVal));
		}
		return points;
	}

	/**
	 * Generates x points for Bezier Curve. x(t)
	 *
	 * @param x0
	 * @param x1
	 * @param x2
	 * @param x3
	 * @param counter
	 * @return Returns the x values for the coordinate for a given t value
	 */
	public double useFunctionX(double x0, double x1, double x2, double x3, double t) {
		double cx = 3 * (x1 - x0);
		double bx = 3 * (x2 - x1) - cx;
		double ax = x3 - x0 - cx - bx;
		double xVal = ax * Math.pow(t, 3) + bx * Math.pow(t, 2) + cx * t + x0;

		return xVal;
	}

	/**
	 * Generates y points for Bezier Curve. y(t)
	 *
	 * @param y0
	 * @param y1
	 * @param y2
	 * @param y3
	 * @param t
	 * @return Returns the y values for the coordinate for a given t parametric value
	 */
	public double useFunctionY(double y0, double y1, double y2, double y3, double t) {
		double cy = 3 * (y1 - y0);
		double by = 3 * (y2 - y1) - cy;
		double ay = y3 - y0 - cy - by;
		double yVal = ay * Math.pow(t, 3) + by * Math.pow(t, 2) + cy * t + y0;

		return yVal;
	}

	/**
	 * Separates coordinates into x and y values get x(t) and y(t) from p(t)
	 */
	public void putPoints() {
		Point point;

		for (int i = 0; i < vector.length; i++) {
			point = vector[i];
			xValues[i] = point.getX();
			yValues[i] = point.getY();
		}
	}

	/**
	 * Used to calculate angles and arc length.
	 * given rectangular coordinates x and y of a point
	 * return its polar coordinates i.e. r =sqrt(x^2+y^2) and theta = arctan(y/x)
	 */
	public void calcPoints() {
		for (int index = 0; index < xPoints.size() - 1; index++) {
			xDelta.add((xPoints.get(index + 1) - xPoints.get(index))); // X(i+1)-X(i)
			yDelta.add((yPoints.get(index + 1) - yPoints.get(index))); // Y(i+1)-Y(i)

			if (xDelta.get(index) == 0) {
				if (yDelta.get(index) > 0)
					angle.add(index, 0.0);
				else if (yDelta.get(index) < 0)
					angle.add(index, 180.0); //replace 180 by Math.PI
			} else if (yDelta.get(index) == 0) {
				if (xDelta.get(index) > 0)
					angle.add(index, 90.0);
				else if (xDelta.get(index) < 0)
					angle.add(index, -90.0);
			} else
				angle.add((Math.toDegrees(Math.atan2(xDelta.get(index), yDelta.get(index))))-118);
			    //angle.add((Math.atan2(xDelta.get(index), yDelta.get(index)))); //use radians directly


			distance += Math.sqrt(Math.pow(xDelta.get(index), 2) + Math.pow(yDelta.get(index), 2));
			hypotenuse.add(index, distance);
		}
		
		recordPathingData(angle,"/home/lvuser/theory6AngleList.txt");
    	//recordPathingData(hypotenuse,"/home/lvuser/theory6HypotenuseList.txt");
		
	}

	/**
	 * @return The arc length of the curve
	 */
	public double findArcLength() {
		return distance;
	}

	/**
	 * Find the angle at an index
	 *
	 * @param index
	 *            Index of angle (0 to number of coordinates)
	 * @return Returns angle in degrees
	 */
	public double findAngle(int index) {
		return angle.get(index);
	}

	/**
	 * Find the hypotenuse length between two coordinates at an index.
	 *
	 * @param index
	 *            Index of hypotenuse segment
	 * @return Returns the hypotenuse length in inches
	 */
	public double findHypotenuse(int index) {
		return hypotenuse.get(index);
	}

	/**
	 * @return Returns all the x values on the Bezier Curve
	 */
	public List<Double> getXPoints() {
		return xPoints;
	}

	/**
	 * @return Returns all the y values on the Bezier Curve
	 */
	public List<Double> getYPoints() {
		return yPoints;
	}

	/**
	 * @return Returns the number of points generated
	 */
	public int size() {
		return size - 1;
	}
	
	public static void recordPathingData(List<Double> data, String filename2) {

    	try { 
    		//DataOutputStream dos= new DataOutputStream(new BufferedOutputStream(new FileOutputStream(filename1))); 
    		//FilterOutputStream fileOutputStream1 = new FilterOutputStream(new BufferedOutputStream(new FileOutputStream(filename1)));
    		FilterOutputStream fileOutputStream = new FilterOutputStream(new BufferedOutputStream(new FileOutputStream(filename2)));
    		
    		//PrintStream printStream1 = new PrintStream(fileOutputStream1);
    		PrintStream printStream = new PrintStream(fileOutputStream);
    		
    		for(int i=0;i<data.size();i++){ 
        		//System.out.println("before"+data.get(i)); 
        		//printStream1.println(dataTime.get(i));
        		//printStream1.println(" ");
        		printStream.println(data.get(i));
        		printStream.println(" ");
        		//dos.writeDouble(data.get(i));
        		//System.out.println("After"+data.get(i)); 
        		} 
    		//printStream1.close();
    		printStream.close();
    		//dos.close(); 
    		
    		} catch (IOException e) { 
    			System.out.println("cant create newDatafile");
    		e.printStackTrace(); 
    		} 
        System.out.println("Data saved.");
    	
    }
	

}
