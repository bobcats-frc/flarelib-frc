package com.bobcats.lib.control;

/**
 * Utility class for calculating subsystem ratios. Returns in terms of REV conversion factor.
 * For CTRE use, take 1/f, the reciprocal instead.
 */
public class SubsystemRatios {
	private SubsystemRatios() {}

	/**
	 * Calculates the Sensor to Mechanism Ratio (STMR) for the elevator subsystem. This ratio is
	 * used to convert sensor readings to mechanism positions. Uses a rope.
	 *
	 * <p>
	 * <b>Units: X/rev</b>
	 *
	 * @param pulleyRadius The radius of the pulley in unit X.
	 * @param reduction    The overall reduction.
	 * @return The calculated STM ratio.
	 */
	public static double elevator_calculateSTMR(double pulleyRadius, double reduction) {
		return pulleyRadius * 2 * Math.PI / reduction;
	}

	/**
	 * Calculates the Sensor to Mechanism Ratio (STMR) for the elevator subsystem. This ratio is
	 * used to convert sensor readings to mechanism positions. Uses a chain.
	 *
	 * <p>
	 * <b>Units: X/rev</b>
	 *
	 * @param chainPitch          The pitch of the chain in unit X.
	 * @param sprocketTeethAmount The number of teeth on the sprocket.
	 * @param reduction           The overall reduction.
	 * @return The calculated STM ratio.
	 */
	public static double elevator_calculateSTMR(double chainPitch, double sprocketTeethAmount, double reduction) {
		return chainPitch * sprocketTeethAmount / reduction;
	}

	/**
	 * Calculates the Sensor to Mechanism Ratio (STMR) for the arm subsystem. This ratio is used to
	 * convert sensor readings to mechanism positions.
	 *
	 * <p>
	 * <b>Units: Degrees/rev</b>
	 *
	 * @param reduction The overall reduction.
	 * @return The calculated STM ratio.
	 */
	public static double arm_calculateSTMR_degs(double reduction) {
		return 360 / reduction;
	}

	/**
	 * Calculates the Sensor to Mechanism Ratio (STMR) for the arm subsystem. This ratio is used to
	 * convert sensor readings to mechanism positions.
	 *
	 * <p>
	 * <b>Units: Rads/rev</b>
	 *
	 * @param reduction The overall reduction.
	 * @return The calculated STM ratio.
	 */
	public static double arm_calculateSTMR_rads(double reduction) {
		return 2 * Math.PI / reduction;
	}
}
