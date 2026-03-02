package com.bobcats.lib.utils;

import com.bobcats.lib.container.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A class that autonomously corrects for a tipping robot by applying a counter-velocity in
 * order to correct the angle.
 */
public class AutonomousSwerveAntiTipping {

	// Roll, pitch controllers
	private final PIDController m_px, m_py;
	private double m_tipThresholdRadsMin, m_tipThresholdRadsMax;
	private double m_speedLimit; // m/s

	private boolean m_isTipping;

	/**
	 * Constructs a new AutonomousSwerveAntiTipping.
	 *
	 * @param correctionalCoeffs  The PID coefficients for the correctional controller. Units are
	 *                            m/rad*s, or (m/s)/rad.
	 * @param tipMinThresholdRads The minimum threshold angle in radians beyond which tipping
	 *                            correction is applied.
	 * @param tipMaxThresholdRads The maximum threshold angle in radians beyond which tipping
	 *                            correction is not applied.
	 * @param speedLimitMps       The maximum speed limit for correctional velocities in m/s.
	 */
	public AutonomousSwerveAntiTipping(PIDConstants correctionalCoeffs, double tipMinThresholdRads,
			double tipMaxThresholdRads, double speedLimitMps) {
		// Set up the PID controllers
		m_px = new PIDController(correctionalCoeffs.kP, correctionalCoeffs.kI, correctionalCoeffs.kD);
		m_py = new PIDController(correctionalCoeffs.kP, correctionalCoeffs.kI, correctionalCoeffs.kD);

		m_tipThresholdRadsMin = tipMinThresholdRads;
		m_tipThresholdRadsMax = tipMaxThresholdRads;
		m_speedLimit = speedLimitMps;

		m_px.setIZone(correctionalCoeffs.iZone);
		m_py.setIZone(correctionalCoeffs.iZone);
	}

	/**
	 * Resets the internal PID controllers for anti-tipping velocities, particularly useful for the
	 * I gains.
	 */
	public void resetControllers() {
		m_px.reset();
		m_py.reset();
	}

	/**
	 * Accounts for anti-tipping by creating an additional chassis speeds countering rotation via
	 * the gyro.
	 *
	 * @param latestGyroReading The latest gyro readings.
	 * @return The chassis speeds adjustment, can also fully replace the commanded chassis speeds.
	 */
	public ChassisSpeeds update(Rotation3d latestGyroReading) {
		ChassisSpeeds corrections = new ChassisSpeeds();
		boolean isTippingX = false, isTippingY = false;

		if (Math.abs(latestGyroReading.getX()) >= m_tipThresholdRadsMin
				&& Math.abs(latestGyroReading.getX()) < m_tipThresholdRadsMax) {
			// A positive rotation around the X axis suggests we need to apply a -Y velocity
			double outputVelocity = -m_py.calculate(latestGyroReading.getX(), 0);
			corrections.vyMetersPerSecond = outputVelocity;
			isTippingX = true;
		}
		if (Math.abs(latestGyroReading.getY()) >= m_tipThresholdRadsMin
				&& Math.abs(latestGyroReading.getY()) < m_tipThresholdRadsMax) {
			// A positive rotation around the Y axis suggests we need to apply a +X velocity
			double outputVelocity = m_px.calculate(latestGyroReading.getY(), 0);
			corrections.vxMetersPerSecond = outputVelocity;
			isTippingY = true;
		}

		// Limit speeds
		if (Math.hypot(corrections.vxMetersPerSecond, corrections.vyMetersPerSecond) > m_speedLimit) {
			double norm = Math.hypot(corrections.vxMetersPerSecond, corrections.vyMetersPerSecond) / m_speedLimit;
			corrections.vxMetersPerSecond /= norm;
			corrections.vyMetersPerSecond /= norm;
		}

		m_isTipping = isTippingX || isTippingY;
		if (!m_isTipping) resetControllers();

		return corrections;
	}

	/**
	 * Sets the angle boundaries for tipping correction.
	 *
	 * @param thrMinAngleRad The minimum threshold angle in radians beyond which tipping correction
	 *                       is applied.
	 * @param thrMaxAngleRad The maximum threshold angle in radians beyond which tipping correction
	 *                       is not applied.
	 */
	public void setAngleBoundaries(double thrMinAngleRad, double thrMaxAngleRad) {
		m_tipThresholdRadsMin = thrMinAngleRad;
		m_tipThresholdRadsMax = thrMaxAngleRad;
	}

	/**
	 * Sets the speed limit for correctional velocities.
	 *
	 * @param speedLimit The maximum speed limit for correctional velocities in m/s.
	 */
	public void setSpeedLimit(double speedLimit) { m_speedLimit = speedLimit; }

	/**
	 * Returns whether the robot is currently tipping.
	 *
	 * @return Whether the robot is currently tipping.
	 */
	public boolean isTipping() { return m_isTipping; }
}
