package org.usfirst.frc3550.Julius2018.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;

public class CommandConsumer {
	
	private Command command;
	private boolean consumed = false;
	
	/**
	 * A command consumer will "consumes" a command, by running it once and 
	 * preventing it from being ran again
	 * @param command The command to consume
	 */
	public CommandConsumer(Command command) {
		this.command = command;
		this.consumed = false;
	}
		
	/**
	 * Consume the command by starting it
	 */
	public boolean consume() {
		if(!consumed) {
//			command.start();
			
			Scheduler.getInstance().add(command);
//			Scheduler.getInstance().
//			System.out.println("CONSUMED");
			consumed = true;
			return true;
		} else {
			return false;
		}
	}
	
	/**
	 * Check if the command has been consumed
	 */
	public boolean isConsumed() {
		return consumed;
	}
	
	/**
	 * @return boolean is the command finished executing
	 */
	public boolean isFinished() {
		return command.isCompleted();
	}
	
	/**
	 * Finish the command, cancelling it and preventing it from ever being consumed
	 */
	public void finish() {
		command.cancel();
		consumed = true;
	}
	
	public void reset() {
		command.cancel();
		consumed = false;
	}
	
	public Command getCommand() {
		return command;
	}

}
