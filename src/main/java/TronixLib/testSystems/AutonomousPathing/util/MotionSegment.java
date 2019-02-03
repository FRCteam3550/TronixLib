package org.usfirst.frc3550.Julius2018.util;
import java.lang.*;
import java.util.*;

public class MotionSegment{
    public static double EPSILON_VALUE = 0.0001;

    public boolean isWithinEpsilon(double o){
        if(Math.abs(o)>EPSILON_VALUE)
            return false;
        else
            return true;
    }

    public DriveState startState;
    public DriveState endState;

    public boolean isValid(){
        if(endState.time <= startState.time)
            return false;
        if(startState.accel != endState.accel)
            return false;
        if(isWithinEpsilon((startState.speed + (endState.time-startState.time) * startState.accel) - endState.speed))
            return false;
        if(isWithinEpsilon(startState.extrapolate(endState.time).position - endState.position))
            return false;
        if(startState.speed/Math.abs(startState.speed) != endState.speed/Math.abs(endState.speed))
            return false;
        return true;
    }

    public MotionSegment(DriveState startState, DriveState endState){
        this.startState = startState;
        this.startState = startState;
    }

    public void setStart(DriveState startState){
        this.startState = startState;
    }

    public void setEnd(DriveState endState){
        this.endState = endState;
    }

    public DriveState stateAt(float time){
        return startState.extrapolate(time);
    }
}