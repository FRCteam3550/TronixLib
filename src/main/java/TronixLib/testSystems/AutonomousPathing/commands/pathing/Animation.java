package org.usfirst.frc3550.Julius2018.commands.pathing;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * An animation has keyframes (robot motions, in the form of commands) that occur at certain points (distances along a path)
 * @author Ryan
 *
 */
public class Animation extends ArrayList<Keyframe> {
	public static final long serialVersionUID = 2;
	
	public Animation(Keyframe... keyframes) {
		super(Arrays.asList(keyframes));
	}
	
	public void animate(double position) {
		for(Keyframe kf : this) {
			kf.animate(position);
		}
	}
	
	public void reset() {
		for(Keyframe kf : this) {
			kf.reset();
		}
	}
}
