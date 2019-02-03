package org.usfirst.frc3550.Julius2018.util;
import java.lang.*;
import java.util.*;

public class MotionProfile{
    public static double EPSILON_VALUE = 0.0001;

    public boolean isWithinEpsilon(double o){
        if(Math.abs(o)>EPSILON_VALUE)
            return false;
        else
            return true;
    }
    public ArrayList<MotionSegment> segments = new ArrayList<MotionSegment>();
    //Check if the motion segments are continuous in position, speed and order
    public MotionProfile(){}

    public boolean isValid(){
        for(int i = 0; i<segments.size(); i++){
            if (!isWithinEpsilon(segments.get(i).endState.position - segments.get(i+1).startState.position))
                return false;
            if (!isWithinEpsilon(segments.get(i).endState.speed - segments.get(i+1).startState.speed))
                return false;
            if (!isWithinEpsilon(segments.get(i).endState.time - segments.get(i+1).startState.time))
                return false;
            if(!(segments.get(i).isValid())){
                return false;
            }
        }
        return true;
    }
    public DriveState driveStateAtTime(double time) {
        for (int i = 0; i < segments.size(); i++) {
            if (segments.get(i).endState.time >= time)
                return segments.get(i).startState.extrapolate(time);
        }
        return segments.get(0).startState;
    }
    public void addSequence(float accel, float duration){
        DriveState initialState = segments.get(segments.size()-1).endState;
        double totalTime = initialState.time + duration;
        initialState.accel = accel;
        MotionSegment addSegment = new MotionSegment(initialState,
                initialState.extrapolate(totalTime));
    }
}