package com.bobcats.lib.control.positions;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

/**
 * An interface allowing users to have a custom enum of subsystem positions.
 *
 * <p>
 * Useful for storing data together, such as arm positions along with elevator positions.
 */
public interface ISubsystemPosition {
	/**
	 * For linear mechanism positions.
	 *
	 * @return The system height.
	 */
	default Distance getHeight() { return Meters.of(0); }

	// For the getAngle methods, we use a subsystem id subsystem in case multiple pivots are
	// present.

	/**
	 * For position-controlled subsystem positions.
	 *
	 * @return The system's angle in degrees.
	 */
	default double getAngle() { return 0; }

	/**
	 * For position-controlled subsystem positions. Used for when there is more than 1 pivot
	 * subsystem.
	 *
	 * @param id The ID of the pivot subsystem.
	 * @return The system's angle in degrees.
	 */
	default double getAngle(String id) {
		return 0;
	}

	/**
	 * For angular velocity-controlled subsystem positions.
	 *
	 * @return The system's RPM.
	 */
	default double getRPM() { return 0; }
}
