package com.bobcats.lib.subsystem.bangbangElevator;

import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.AutoLog;

/**
 * The main superclass interface for elevator motor (hardware) interaction IO classes.
 */
public interface BangBangElevatorIO {

	/**
	 * The inputs class for the bang-bang controlled Elevator Subsystem.
	 *
	 * <p>
	 * Note: the position and velocity values are the motor values, not output values.
	 */
	@AutoLog
	public static class BangBangElevatorIOInputs {
		public boolean motorConnected = true;
		public double velocityRPM = 0.0;
		public double positionRevs = 0.0;
		public double appliedVoltage = 0.0;
		public double supplyCurrentAmps = 0.0;
		public double statorCurrentAmps = 0.0;
		public double temperatureCelsius = 0.0;
	}

	/**
	 * Updates the given elevator inputs.
	 *
	 * @param inputs The inputs.
	 */
	public default void updateInputs(BangBangElevatorIOInputs inputs) {}

	/**
	 * Applies the given voltage to the motors.
	 *
	 * @param volts The voltage.
	 */
	public default void runVolts(double volts) {}

	/**
	 * Runs the bang-bang controller via torque-current (stator current). This is only supported on
	 * FOC-enabled CTRE hardware.
	 *
	 * @param amps The current in Amps.
	 */
	public default void runTorqueCurrent(double amps) {
		DriverStation.reportWarning("Can't run torque-current on unsupported hardware", true);
	}
}
