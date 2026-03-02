package com.bobcats.lib.control.feedforward;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Extension of WPILib's ArmFeedforward to include feedforward compensation for external
 * vertical accelerations acting on the arm.
 */
public class LibArmFeedforward {
	private ArmFeedforward m_feedforward;
	private double m_dt;
	private double m_center, m_mass, m_gearing, m_numMotors, m_efficiency;
	private DCMotor m_gearbox;

	/**
	 * Constructs a new LibArmFeedforward.
	 *
	 * @param kS                 The static gain, in Volts.
	 * @param kG                 The gravity gain, in Volts.
	 * @param kV                 The velocity gain, in V*s/rad.
	 * @param kA                 The acceleration gain, in V*s^2/rad.
	 * @param dt                 The time step between controller updates, in seconds.
	 * @param armMass            The mass of the arm, in kilograms.
	 * @param armCOMRadiusMeters The distance from the arm's pivot to its center of mass, in
	 *                           meters.
	 * @param gearing            The reduction of the arm.
	 * @param numMotors          The number of motors driving the arm.
	 * @param gearbox            The DCMotor gearbox model. (do not include the reduction here)
	 * @param efficiency         The efficiency of the drivetrain on a scale of 0 to 1.
	 */
	public LibArmFeedforward(double kS, double kG, double kV, double kA, double dt, double armMass,
			double armCOMRadiusMeters, double gearing, double numMotors, DCMotor gearbox, double efficiency) {
		m_feedforward = new ArmFeedforward(kS, kG, kV, kA, dt);
		m_dt = dt;
		m_center = armCOMRadiusMeters;
		m_mass = armMass;
		m_gearing = gearing;
		m_numMotors = numMotors;
		m_efficiency = efficiency;
		m_gearbox = gearbox;
	}

	/**
	 * Calculates the required feedforward voltage for the arm at a given position, velocity,
	 * acceleration, and external vertical acceleration.
	 *
	 * @param positionRadians             The current position of the arm, in radians.
	 * @param velocityRadPerSec           The current velocity of the arm, in rad/s.
	 * @param accelerationRadPerSecSq     The current acceleration of the arm, in rad/s^2.
	 * @param verticalAccelMetersPerSecSq The external vertical acceleration acting on the arm, in
	 *                                    m/s^2.
	 * @return The required feedforward in Volts.
	 */
	public double calculate(double positionRadians, double velocityRadPerSec, double accelerationRadPerSecSq,
			double verticalAccelMetersPerSecSq) {
		// Simple gravity feedforward
		double ffVolts = m_feedforward.calculateWithVelocities(positionRadians, velocityRadPerSec,
				velocityRadPerSec + accelerationRadPerSecSq * m_dt);

		// Torque exerted on the arm due to chasss acceleration: T = M_arm * a_chassis * cosA * r_com
		// Torque exerted on the shaft: T = T_arm / G, where G is the gear ratio
		// To counter this torque, we need to apply an additional feedforward voltage:
		// V = F_v * r_com * cosA * R / kT
		double torqueArm = m_mass * verticalAccelMetersPerSecSq * Math.cos(positionRadians) * m_center;
		double torqueShaftPer = torqueArm / (m_gearing * m_numMotors);
		double perMotorResistance = m_gearbox.rOhms * m_numMotors;
		double accelFeedforward = torqueShaftPer * perMotorResistance / (m_gearbox.KtNMPerAmp * m_efficiency);

		return ffVolts + accelFeedforward;
	}
}
