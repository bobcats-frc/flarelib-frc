package com.bobcats.lib.math;

/** A physics utility class. */
public class PhysicsUtil {
	private PhysicsUtil() {}

	/** The graviational acceleration of the earth in m/s^2. */
	public static double kG = 9.80665;

	/**
	 * Computes the air density.
	 *
	 * <p>
	 * Note: Air density can be calculated via:
	 * <p>
	 * <code>S = 611.21 * exp(17.502 * T / (240.97 + T))
	 * Rho_air = (101325 * (1 - 0.0065 * Y / 288.15)^5.255877 - H * S) / (287.05 * (T +
	 * 273.15)) + H * S / (461.5 * (T + 273.15))</code>
	 * <p>
	 * where T is the temperature in Celsius, Y is the altitude in meters, and H is the relative
	 * humidity in range [0, 1].
	 *
	 * @param temp The temperature, in Celsius.
	 * @param alt  The altitude, in meters.
	 * @param rel  The relative humidity, in the range [0, 100]%.
	 * @return The air density, in kg/m^3.
	 */
	public static double computeAirDensity(double temp, double alt, double rel) {
		rel /= 100;
		double sat = 611.21 * Math.exp(17.502 * temp / (240.97 + temp));
		return (101325 * Math.pow((1 - 0.0065 * alt / 288.15), 5.255877) - rel * sat) / (287.05 * (temp + 273.15))
				+ rel * sat / (461.5 * (temp + 273.15));
	}
}
