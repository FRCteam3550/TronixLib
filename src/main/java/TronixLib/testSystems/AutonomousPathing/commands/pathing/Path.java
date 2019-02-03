package org.usfirst.frc3550.Julius2018.commands.pathing;

import java.util.ArrayList;
import java.util.Arrays;

public class Path extends ArrayList<PathSegment> {
	public static final long serialVersionUID = 340; //Because why not?

	public Path(PathSegment... paths) { // Path receive a variable number of arguments. paths is an array of PathSegment
        super(Arrays.asList(paths));  // return a fixed-size list backed by array paths: array of pathSegments
    }

    public double getTotalLength() {
        // Returns the sum of the lengths of each Path
        return this.stream().mapToDouble(PathSegment::getLength).sum();
    }
    
    public double getTotalOfCompletedPaths(double distance) {
    	System.out.println("PATH getPathTotal INIT");
    	PathSegment current = getPathSegmentAtDistance(distance);
    	Path previous = new Path();
    	for(PathSegment p : this) {
    		if (current.equals(p)) {
    			System.out.println("LenghtCompleted1 in PathFile: "+ previous.getTotalLength());
    			System.out.println("PATH getPathTotal END1");
    			return previous.getTotalLength();
    			//System.out.println("Lenght i: "+ previous.getTotalLength());
    		} else {
    			previous.add(p);
    		}
    	}
    	System.out.println("LenghtCompleted2 in PathFile: "+previous.getTotalLength());
    	System.out.println("PATH getPathTotal END2");
    	return previous.getTotalLength();
    	//System.out.println("Lenght i: "+previous.getTotalLength());
    }

    /**
     * ex: if you traveled past the first Path's length, it returns the second (or third, or fourth...)
     */
    public PathSegment getPathSegmentAtDistance(double distance) {
    	System.out.println("PATH getPathDist INIT");
        int i = 0;
        while (distance >= 0 && i < this.size()) {
        	System.out.println("PathDistance in PathFile: " + distance);
        	System.out.println("ThisSize i PathFile: " + this.size());
        	System.out.println("Lenght in PathFile: "+ this.get(i).getLength());
            distance -= this.get(i).getLength();
            System.out.println("i in PathFile: " + i);
            System.out.println("PathDistance in PathFile: " + distance);
            i++;
        }
        System.out.println("PATH getPathDist END");
        return this.get(i - 1);
    }

}
