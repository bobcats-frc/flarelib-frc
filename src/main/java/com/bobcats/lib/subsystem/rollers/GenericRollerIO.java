package com.bobcats.lib.subsystem.rollers;

import org.littletonrobotics.junction.AutoLog;

/**
 * The main superclass interface for roller motor (hardware) interaction IO classes.
 */
public interface GenericRollerIO {
	/**
	 * The inputs class for the Generic Rollers.
	 * <p>
	 * Note: the position and velocity values are the motor values, not output values.
	 */
	@AutoLog
	public static class GenericRollerIOInputs {
		public boolean motorConnected = true;
		public double rpm = 0.0;
		public double positionRevs = 0.0;
		public double appliedVoltage = 0.0;
		public double supplyCurrentAmps = 0.0;
		public double statorCurrentAmps = 0.0;
		public double temperatureCelsius = 0.0;
	}

	/**
	 * Updates the given roller inputs.
	 *
	 * @param inputs The inputs.
	 */
	public default void updateInputs(GenericRollerIOInputs inputs) {}

	/**
	 * Sets the voltage of the motors.
	 *
	 * @param volts The voltage.
	 */
	public default void runVolts(double volts) {}
}
