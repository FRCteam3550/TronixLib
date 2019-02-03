package org.usfirst.frc3550.Julius2018.util;

import java.lang.*;
import java.util.*;


public class TrapezProfile{

    double slope = 0;
    double distance = 0;
    double initialSpeed = 0;
    double maxSpeed = 10;
    double accelTriangleSize = 0;

    boolean isCruising = true;

    MotionProfile generatedProfile;
    ArrayList<DriveState> stateList;

    public void stateListToMotionProfile(){
        MotionProfile generatedProfile = new MotionProfile();
        for(int i=0; i+1 < stateList.size(); i++){
            generatedProfile.segments.add(new MotionSegment(stateList.get(i),
                    stateList.get(i+1)));
        }
    }
    public TrapezProfile(){
    }
    public MotionProfile getGeneratedProfile(){
        if(generatedProfile.isValid())
            return generatedProfile;
        else{
            stateListToMotionProfile();
            return generatedProfile;
        }
    }
    public void createTrapezProfile(double accelSlope, double distance, double maxSpeed, double currentSpeed){
        slope = accelSlope;
        this.initialSpeed = currentSpeed;
        this.distance = distance;
        this.maxSpeed = maxSpeed;
        DriveState initialState = new DriveState(0, slope, initialSpeed, 0);
        stateList.add(initialState);
        accelTriangleSize = stateList.get(0).extrapolate(stateList.get(0).timeToSpeed(maxSpeed)).position;
        if(distance < 2*accelTriangleSize){
            isCruising = false;
        }
        if(isCruising){
            stateList.add(initialState.extrapolate(initialState.timeToSpeed(maxSpeed)));
            //We are now at a distance equal to accelTriangleSize
            //We need to cruise until there is exactly enough distance left to brake
            stateList.get(1).accel = 0;
            stateList.add(stateList.get(1).extrapolate(stateList.get(1).timeToPos(distance-accelTriangleSize)));
            //We now need to brake until stoped and at position
            stateList.get(2).accel = -slope;
            stateList.add(stateList.get(2).extrapolate(stateList.get(2).timeToSpeed(0)));
            //Done
        }
        if(!isCruising){
            stateList.add(initialState.extrapolate(initialState.timeToPos(distance/2)));
            //We are now at max speed, and we now need to immediately brake as there is not
            //  enough space to reach maxSpeed
            stateList.get(1).accel = -slope;
            stateList.add(stateList.get(1).extrapolate(stateList.get(1).timeToPos(distance)));
            //Done
        }
        stateListToMotionProfile();
    }
    public TrapezProfile(double accelSlope, double distance, double maxSpeed, double currentSpeed){
       createTrapezProfile(accelSlope, distance, maxSpeed, currentSpeed);
    }
    public MotionProfile generateProfile(double accelSlope, double distance, double maxSpeed, double currentSpeed){
        createTrapezProfile(accelSlope, distance, maxSpeed, currentSpeed);
        return this.generatedProfile;
    }
//    public double speedAtTime(double time){
//        DriveState targetDriveState;
//        for(int i = 1; i<stateList.size(); i++){
//            if(stateList.get(i).time > time){
//                targetDriveState = stateList.get(i-1);
//            }
//        }
//        return targetDriveState.speedAtTime(time);
//    }
//    public double posAtTime(double time){
//        DriveState targetDriveState;
//        for(int i = 1; i<stateList.size(); i++){
//            if(stateList.get(i).time > time){
//                targetDriveState = stateList.get(i-1);
//            }
//        }
//        return targetDriveState.extrapolate(time).speed;
//    }
//    public double accelAtTime(double time){
//        DriveState targetDriveState;
//        for(int i = 1; i<stateList.size(); i++){
//            if(stateList.get(i).time > time){
//                targetDriveState = stateList.get(i-1);
//            }
//        }
//        return targetDriveState.accel;
//    }
}