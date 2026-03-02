package com.bobcats.lib.control;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

/** Utility class for basic kinematics related to wheel rotations. */
public class RotationalKinematics {
	private RotationalKinematics() {
		throw new UnsupportedOperationException("Cannot instantiate static class RotationalKinematics");
	}

	/**
	 * Returns the distance travelled by wheel, given the reduction and revolutions.
	 *
	 * @param diameter  The diameter of the wheel.
	 * @param reduction The reduction from the motor to the wheel.
	 * @param revs      The number of motor revolutions made.
	 * @return The distance.
	 */
	public static Distance distanceTravelledByWheel(Distance diameter, double reduction, double revs) {
		double circumferenceMeters = diameter.in(Meters) * Math.PI;
		double wheelRevs = revs / reduction;
		return Meters.of(circumferenceMeters * wheelRevs);
	}

	/**
	 * Returns the number of motor revolutions, given the reduction and distance travelled.
	 *
	 * @param wheelDiameter The diameter of the wheel.
	 * @param travelled     The distance travelled.
	 * @param reduction     The reudction from the motor to the wheel.
	 * @return The motor revolutions.
	 */
	public static double motorRevsFromDistanceTravelledByWheel(Distance wheelDiameter, Distance travelled,
			double reduction) {
		double wheelRevs = travelled.in(Meters) / (wheelDiameter.in(Meters) * Math.PI);
		return wheelRevs * reduction;
	}
}
