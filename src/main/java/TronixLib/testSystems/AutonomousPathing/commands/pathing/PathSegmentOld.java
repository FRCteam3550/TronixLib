package org.usfirst.frc3550.Julius2018.commands.pathing;

import java.util.function.Function;

public class PathSegmentOld {

	private Function<Double, Double> derivative; // the Path derivative and the path lenght
    private double length;
    //private function derivate is a function that accepts a single Double input and
    //returns a Double valued ouput
    public PathSegmentOld(Function<Double, Double> derivative, double length) {
        this.derivative = derivative;
        this.length = length;
    }

    public Function<Double, Double> getDerivative() {  // getDerivative is a function that returns a function that accepts 
        return derivative; //a Double as input and returns a Double
    }

    public double getLength() {
        return length;
    }

    
}
