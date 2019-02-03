package org.usfirst.frc3550.Julius2018.commands.pathing;

import org.usfirst.frc3550.Julius2018.commands.CommandConsumer;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Keyframes are robot motions which occur at certain points
 * @author Ryan
 *
 */
public class Keyframe {
	private CommandConsumer consumer;
	private double point;
	private boolean done = false;
	
	public Keyframe(Command command, double point) {
		consumer = new CommandConsumer(command);
		this.point = point;
	}
	
	/**
	 * Consumes the command, playing the animation.
	 */
	private void play() {
		if(!done) {
			consumer.consume();
		}
		if(consumer.isConsumed()) {
			done = true;
		}
	}
	
	/**
	 * @return whether the keyframe is still queued (hasn't been played yet)
	 */
	public boolean queued() {
		if(done || consumer.isConsumed()) {
			return false;
		}
		return true;
	}
	
	/**
	 * Keyframes are continously animated. They will only play when the certain point has been reached
	 * @param position The robot's position
	 */
	public void animate(double position) {
		if(position >= point && queued()) {
			play();
		}
	}
	
	public void reset() {
		consumer.reset();
		done = false;
	}
	
	public void setParents(CommandGroup parent) {
		
	}
	
	public CommandConsumer getCommandConsumer() {
		return consumer;
	}
}
