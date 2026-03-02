package com.bobcats.lib.control.shooter.data;

import com.bobcats.lib.math.BilinearInterpolator2D;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * A descriptor for a shooter subsystem, containing all the necessary parameters to calculate
 * the angle, RPM and yaw of the shooter, given environmental factors.
 */
public class ShooterDescriptor {
	private Transform3d m_robotToTurret3d;
	private double m_rollerRadius;

	private double m_shotDelay;
	private double m_barrelLength;

	private double m_minTurretAngle, m_maxTurretAngle;
	private double m_minAllowedDistance, m_maxAllowedDistance;

	// Calculation components
	private BilinearInterpolator2D m_angleInterpolator;
	private BilinearInterpolator2D m_rpmInterpolator;
	private BilinearInterpolator2D m_flightTimeInterpolator;

	/**
	 * Constructs a new ShooterDescriptor.
	 *
	 * @param builder The parameter Builder instance.
	 */
	private ShooterDescriptor(Builder builder) {
		m_robotToTurret3d = builder.m_robotToTurret3d;
		m_rollerRadius = builder.m_rollerRadius;

		m_minTurretAngle = builder.m_minTurretAngle;
		m_maxTurretAngle = builder.m_maxTurretAngle;
		m_shotDelay = builder.m_shotDelay;
		m_barrelLength = builder.m_barrelLength;
		m_minAllowedDistance = builder.m_minAllowedDistance;
		m_maxAllowedDistance = builder.m_maxAllowedDistance;
		m_angleInterpolator = builder.m_angleInterpolator;
		m_rpmInterpolator = builder.m_rpmInterpolator;
		m_flightTimeInterpolator = builder.m_flightTimeInterpolator;
	}

	/** A builder class for ShooterDescriptor parameters. */
	public static class Builder {
		private final Transform3d m_robotToTurret3d;
		private final double m_rollerRadius;

		private double m_shotDelay = 0; //
		private double m_barrelLength = 0.15; //

		private double m_minTurretAngle = 180, m_maxTurretAngle = -180; //
		private double m_minAllowedDistance = 0, m_maxAllowedDistance = Double.POSITIVE_INFINITY; //

		// Calculation components
		private BilinearInterpolator2D m_angleInterpolator;
		private BilinearInterpolator2D m_rpmInterpolator;
		private BilinearInterpolator2D m_flightTimeInterpolator;

		/**
		 * Constructs a new Builder.
		 *
		 * @param rollerRadius       The radius of the rollers, in meters.
		 * @param robotToTurretPivot The transform from the robot to the turret pivot point.
		 */
		public Builder(double rollerRadius, Transform3d robotToTurretPivot) {
			m_rollerRadius = rollerRadius;
			m_robotToTurret3d = robotToTurretPivot;
		}

		/**
		 * Sets the yaw turret angle range for the shooter, in degrees.
		 *
		 * @param minDeg The minimum angle, in degrees. Ideally should be negative.
		 * @param maxDeg The maximum angle in degrees. Ideally should be positive.
		 * @return The builder instance for chaining.
		 */
		public Builder turretAngleRange(double minDeg, double maxDeg) {
			m_minTurretAngle = minDeg;
			m_maxTurretAngle = maxDeg;
			return this;
		}

		/**
		 * Sets the average delay between the feeders running and the shot being fired, in seconds.
		 * It's recommended to update shooter parameters even right before the shot is fired, in which
		 * case this delay should only be non-zero if the delay is less than the average loop period
		 * (all software lag included), where the new parameters can't be calculated in time. If the
		 * shot is pre-calculated and not updated, set this to the raw time delay between the feeder
		 * running and the shot being fired.
		 *
		 * @param seconds The delay, in seconds.
		 * @return The builder instance for chaining.
		 */
		public Builder shotDelay(double seconds) {
			m_shotDelay = seconds;
			return this;
		}

		/**
		 * Sets the length of the barrel from the pivot point to the exit point.
		 *
		 * @param length The length, in meters.
		 * @return The builder instance for chaining.
		 */
		public Builder barrelLength(double length) {
			m_barrelLength = length;
			return this;
		}

		/**
		 * Adds a required fully-empirical dataset to the shooter descriptor.
		 * <p>
		 *
		 * Example arrays:
		 *
		 * <pre>
		 * <code>
		 * // Arrays, must be sorted in ascending order
		 * double[] horizontalDistances = { 2.0, 3.0, 4.0, 5.0 }; // meters
		 * double[] heightDiffs = { -0.5, 0.0, +0.5, +1.0 }; // meters, relative to ground (not pivot)
		 * double[][] angleGrid = {
		 *         // -----------------> HeightDiff (-0.5 -> +1.0)
		 *         { 45, 40, 35, 30 }, // at d=2.0
		 *         { 42, 38, 33, 28 }, // at d=3.0
		 *         { 39, 35, 31, 27 }, // at d=4.0
		 *         { 37, 33, 29, 25 }, // at d=5.0
		 * };
		 * double[][] rpmGrid = {
		 *         // -----------------> HeightDiff (-0.5 -> +1.0)
		 *         { 4500, 4000, 3500, 3000 }, // at d=2.0
		 *         { 4200, 3800, 3300, 2800 }, // at d=3.0
		 *         { 3900, 3500, 3100, 2700 }, // at d=4.0
		 *         { 3700, 3300, 2900, 2500 }, // at d=5.0
		 * };
		 * double[][] flightTimeGrid = {
		 *         // -----------------> HeightDiff (-0.5 -> +1.0)
		 *         { 0.5, 0.55, 0.6, 0.65 }, // at d=2.0
		 *         { 0.6, 0.65, 0.7, 0.75 }, // at d=3.0
		 *         { 0.7, 0.75, 0.8, 0.85 }, // at d=4.0
		 *         { 0.8, 0.85, 0.9, 0.95 }, // at d=5.0
		 * };
		 * </code> </pre>
		 *
		 * <p>
		 * <i><b>Notes:</b></i>
		 * <p>
		 * <i> <b> - The emprical data must be measured from the robot's center, not the barrel. </b>
		 * </i>
		 * <p>
		 * <i> <b> - In order to use the interpolators correctly,
		 * {@link #setDistanceLimits(double, double)} must be called.
		 * <p>
		 * <i> <b> - The grids must be sorted in ascending order. </b> </i>
		 * <p>
		 * <i> <b> - The data should be large enough to avoid extrapolation due to out-of-bounds data,
		 * which may lead to significant inacuracies. </b> </i>
		 * <p>
		 * <i> <b> - The RPM speeds use the roller RPMs, not the motor. </b> </i>
		 *
		 * @param angleInterpolator      The bilinear interpolator for the angles, as described above.
		 * @param rpmInterpolator        The bilinear interpolator for the RPMs, as described above.
		 * @param flightTimeInterpolator The bilinear interpolator for the flight times, as described
		 *                               above.
		 * @return The builder instance for chaining.
		 */
		public Builder dataSet(BilinearInterpolator2D angleInterpolator, BilinearInterpolator2D rpmInterpolator,
				BilinearInterpolator2D flightTimeInterpolator) {
			m_angleInterpolator = angleInterpolator;
			m_rpmInterpolator = rpmInterpolator;
			m_flightTimeInterpolator = flightTimeInterpolator;
			return this;
		}

		/**
		 * Sets the distance limits for the empirical data sets, in meters. This is required to use the
		 * interpolators correctly.
		 *
		 * @param minDistanceMeters The minimum distance in meters.
		 * @param maxDistanceMeters The maximum distance in meters.
		 * @return The builder instance for chaining.
		 */
		public Builder setDistanceLimits(double minDistanceMeters, double maxDistanceMeters) {
			m_minAllowedDistance = minDistanceMeters;
			m_maxAllowedDistance = maxDistanceMeters;
			return this;
		}

		/**
		 * Builds the ShooterDescriptor instance with the provided parameters.
		 *
		 * @return A new ShooterDescriptor instance.
		 */
		public ShooterDescriptor build() {
			return new ShooterDescriptor(this);
		}
	}

	// Accessors
	public Transform3d getRobotToTurret() { return m_robotToTurret3d; }

	public double getRollerRadius() { return m_rollerRadius; }

	public double getShotDelay() { return m_shotDelay; }

	public double getBarrelLength() { return m_barrelLength; }

	public double getMinTurretAngle() { return m_minTurretAngle; }

	public double getMaxTurretAngle() { return m_maxTurretAngle; }

	public double getMinDistance() { return m_minAllowedDistance; }

	public double getMaxDistance() { return m_maxAllowedDistance; }

	public BilinearInterpolator2D getAngleInterpolator() { return m_angleInterpolator; }

	public BilinearInterpolator2D getRPMInterpolator() { return m_rpmInterpolator; }

	public BilinearInterpolator2D getFlightTimeInterpolator() { return m_flightTimeInterpolator; }
}
