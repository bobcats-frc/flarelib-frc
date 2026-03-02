package com.bobcats.lib.auto;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * A functional interface for building custom routines, whose sequencing patterns may differ
 * from season to season and need special command sequencing.
 */
@FunctionalInterface
public interface CustomRoutineBuilder {
	/**
	 * Returns the custom auto routine as a command.
	 *
	 * @return The custom auto routine as a command.
	 */
	Command getRoutine();
}
