package org.usfirst.frc3550.Julius2018.commands.pathing;

import java.util.function.Function;

public class PathSegment {

	public static class Vec2 {
		public int x;
		public int y;
		public Vec2(int x, int y) {
			this.x = x;
			this.y = y;
		}
		@Override
		public String toString() {
			return "(" + x + ", " + y + ")";
		}
	}
	
	private Function<Double, Double> derivative;
    private double length;

    public PathSegment(Function<Double, Double> derivative, double length) {
        this.derivative = derivative;
        this.length = length;
        this.curvature = t -> {
        	System.err.println("Using curvature from a segment constructed in the old style");
        	return 0.0001;
        };
    }
    
    private Function<Double, Double> dx; //  first derivative for x
    private Function<Double, Double> dy; //  first derivative for x
    private Function<Double, Double> dx2; // second derivative for x
    private Function<Double, Double> d2y; // second derivative for y
    private Function<Double, Double> curvature; // curvature at a given t
    
    /**
     * Alternative constructor that takes each point for the curve
     * @param start
     * @param mid1
     * @param mid2
     * @param end
     * @param length
     */
    public PathSegment(Vec2 start, Vec2 mid1, Vec2 mid2, Vec2 end, double length) {
    	// first derivatives of x and y
    	dx = t -> {
    		return (double) (
    				3 * (end.x + 3 * mid1.x - 3 * mid2.x - start.x) * Math.pow(t, 2) - 
    				6 * (2 * mid1.x - mid2.x - start.x) * t +
    				3 * mid1.x - 3 * start.x
    				);
    	};
    	dy = t -> {
    		return (double) (
    				3 * (end.y + 3 * mid1.y - 3 * mid2.y - start.y) * Math.pow(t, 2) - 
    				6 * (2 * mid1.y - mid2.y - start.y) * t +
    				3 * mid1.y - 3 * start.y
    				);
    	};
    	
    	// first derivative of the bezier. slope of line tangent at t
    	this.derivative = t -> {
    		return dy.apply(t) / dx.apply(t);
    	};
    	
    	// second derivatives. to be used for curvature
    	dx2 = t -> {
    		return (double) (6 * (end.x + 3 * mid1.x - 3 * mid2.x - start.x) * t - 6 * (2 * mid1.x - mid2.x - start.x));
    	};
    	d2y = t -> {
    		return (double) (6 * (end.y + 3 * mid1.y - 3 * mid2.y - start.y) * t - 6 * (2 * mid1.y - mid2.y - start.y));
    	};
    	
    	// https://en.wikipedia.org/wiki/Curvature#Local_expressions
    	curvature = t -> {
    		System.out.println(Math.pow(Math.pow(dx.apply(t), 2) + Math.pow(dy.apply(t), 2), 1.5));
    		return Math.abs(dx.apply(t) * d2y.apply(t) - dy.apply(t) * dx2.apply(t)) 
    				/ Math.pow(Math.pow(dx.apply(t), 2) + Math.pow(dy.apply(t), 2), 1.5);
    	};
    	this.length = length;
    }

    public Function<Double, Double> getDerivative() {
        return derivative;
    }

    public double getLength() {
        return length;
    }
    
    public Function<Double, Double> getCurvature() {
    	return curvature;
    }
    
}
