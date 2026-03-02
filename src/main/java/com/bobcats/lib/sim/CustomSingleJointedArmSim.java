// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.bobcats.lib.sim;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import org.ironmaple.simulation.SimulatedArena.Simulatable;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

/** Represents a simulated single jointed arm mechanism. */
public class CustomSingleJointedArmSim extends LinearSystemSim<N2, N1, N2> implements Simulatable {

	// TODO issue, internal measurements receive noised measurements

	// The gearbox for the arm.
	private final DCMotor m_gearbox;

	// The gearing between the motors and the output.
	private final double m_gearing;

	// The length of the arm.
	private final double m_armLenMeters;

	// The minimum angle that the arm is capable of.
	private final double m_minAngle;

	// The maximum angle that the arm is capable of.
	private final double m_maxAngle;

	// Whether the simulator should simulate gravity.
	private final boolean m_simulateGravity;

	private final int m_numMotors;

	private double m_supplyLim, m_statorLim;

	private double m_lastTick = -1;
	private double m_requestedVoltage;

	/**
	 * Creates a simulated arm mechanism.
	 *
	 * @param plant              The linear system that represents the arm. This system can be
	 *                           created with
	 *                           {@link edu.wpi.first.math.system.plant.LinearSystemId#createSingleJointedArmSystem(DCMotor, double, double)}.
	 * @param gearbox            The type of and number of motors in the arm gearbox.
	 * @param gearing            The gearing of the arm (numbers greater than 1 represent
	 *                           reductions).
	 * @param armLengthMeters    The length of the arm.
	 * @param minAngleRads       The minimum angle that the arm is capable of.
	 * @param maxAngleRads       The maximum angle that the arm is capable of.
	 * @param simulateGravity    Whether gravity should be simulated or not.
	 * @param startingAngleRads  The initial position of the Arm simulation in radians.
	 * @param numMotors          The number of motors driving the system.
	 * @param measurementStdDevs The standard deviations of the measurements. Can be omitted if no
	 *                           noise is desired. If present must have 1 element for position.
	 */
	@SuppressWarnings("this-escape")
	public CustomSingleJointedArmSim(LinearSystem<N2, N1, N2> plant, DCMotor gearbox, double gearing,
			double armLengthMeters, double minAngleRads, double maxAngleRads, boolean simulateGravity,
			double startingAngleRads, int numMotors, double... measurementStdDevs) {
		super(plant, measurementStdDevs);
		m_gearbox = gearbox;
		m_gearing = gearing;
		m_armLenMeters = armLengthMeters;
		m_minAngle = minAngleRads;
		m_maxAngle = maxAngleRads;
		m_simulateGravity = simulateGravity;
		m_numMotors = numMotors;

		SimulatedBattery.addElectricalAppliances(
				() -> Amps.of(Math.abs(getCurrentDrawAmps() * m_u.get(0, 0) / getBatteryVolts())));

		setState(startingAngleRads, 0.0);
	}

	/**
	 * Creates a simulated arm mechanism.
	 *
	 * @param gearbox            The type of and number of motors in the arm gearbox.
	 * @param gearing            The gearing of the arm (numbers greater than 1 represent
	 *                           reductions).
	 * @param jKgMetersSquared   The moment of inertia of the arm; can be calculated from CAD
	 *                           software.
	 * @param armLengthMeters    The length of the arm.
	 * @param minAngleRads       The minimum angle that the arm is capable of.
	 * @param maxAngleRads       The maximum angle that the arm is capable of.
	 * @param simulateGravity    Whether gravity should be simulated or not.
	 * @param startingAngleRads  The initial position of the Arm simulation in radians.
	 * @param numMotors          The number of motors driving the system.
	 * @param measurementStdDevs The standard deviations of the measurements. Can be omitted if no
	 *                           noise is desired. If present must have 1 element for position.
	 */
	public CustomSingleJointedArmSim(DCMotor gearbox, double gearing, double jKgMetersSquared, double armLengthMeters,
			double minAngleRads, double maxAngleRads, boolean simulateGravity, double startingAngleRads, int numMotors,
			double... measurementStdDevs) {
		this(LinearSystemId.createSingleJointedArmSystem(gearbox, jKgMetersSquared, gearing), gearbox, gearing,
				armLengthMeters, minAngleRads, maxAngleRads, simulateGravity, startingAngleRads, numMotors,
				measurementStdDevs);
	}

	/**
	 * Sets the arm's state. The new angle will be limited between the minimum and maximum allowed
	 * limits.
	 *
	 * @param angleRadians      The new angle in radians.
	 * @param velocityRadPerSec The new angular velocity in radians per second.
	 */
	public final void setState(double angleRadians, double velocityRadPerSec) {
		setState(VecBuilder.fill(MathUtil.clamp(angleRadians, m_minAngle, m_maxAngle), velocityRadPerSec));
	}

	/**
	 * Returns whether the arm would hit the lower limit.
	 *
	 * @param currentAngleRads The current arm height.
	 * @return Whether the arm would hit the lower limit.
	 */
	public boolean wouldHitLowerLimit(double currentAngleRads) {
		return currentAngleRads <= this.m_minAngle;
	}

	/**
	 * Returns whether the arm would hit the upper limit.
	 *
	 * @param currentAngleRads The current arm height.
	 * @return Whether the arm would hit the upper limit.
	 */
	public boolean wouldHitUpperLimit(double currentAngleRads) {
		return currentAngleRads >= this.m_maxAngle;
	}

	/**
	 * Returns whether the arm has hit the lower limit.
	 *
	 * @return Whether the arm has hit the lower limit.
	 */
	public boolean hasHitLowerLimit() {
		return wouldHitLowerLimit(getAngleRads());
	}

	/**
	 * Returns whether the arm has hit the upper limit.
	 *
	 * @return Whether the arm has hit the upper limit.
	 */
	public boolean hasHitUpperLimit() {
		return wouldHitUpperLimit(getAngleRads());
	}

	/**
	 * Returns the current arm angle.
	 *
	 * @return The current arm angle.
	 */
	public double getAngleRads() { return getOutput(0); }

	/**
	 * Returns the current arm velocity.
	 *
	 * @return The current arm velocity.
	 */
	public double getVelocityRadPerSec() { return getOutput(1); }

	/**
	 * Returns the arm current draw.
	 *
	 * @return The arm current draw.
	 */
	public double getCurrentDrawAmps() {
		// Reductions are greater than 1, so a reduction of 10:1 would mean the motor is
		// spinning 10x faster than the output
		var motorVelocity = m_x.get(1, 0) * m_gearing;
		return m_gearbox.getCurrent(motorVelocity, m_u.get(0, 0)) * Math.signum(m_u.get(0, 0));
	}

	/**
	 * Sets the input voltage for the arm.
	 *
	 * @param volts The input voltage.
	 */
	public void setInputVoltage(double volts) {
		// setInput(volts);
		// clampInput(RobotController.getBatteryVoltage());
		m_requestedVoltage = volts;
	}

	/**
	 * Calculates a rough estimate of the moment of inertia of an arm given its length and mass.
	 *
	 * @param lengthMeters The length of the arm.
	 * @param massKg       The mass of the arm.
	 * @return The calculated moment of inertia.
	 */
	public static double estimateMOI(double lengthMeters, double massKg) {
		return 1.0 / 3.0 * massKg * lengthMeters * lengthMeters;
	}

	/**
	 * Updates the state of the arm.
	 *
	 * @param currentXhat The current state estimate.
	 * @param u           The system inputs (voltage).
	 * @param dtSeconds   The time difference between controller updates.
	 */
	@Override
	protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {

		// Desaturate voltage to stay within current limits
		double appliedVolts = HardwareSimUtils.desaturateVoltage(m_gearbox, getVelocityRadPerSec() * m_gearing,
				getBatteryVolts(), m_requestedVoltage, u.get(0, 0), m_statorLim * m_numMotors,
				m_supplyLim * m_numMotors);
		m_u.set(0, 0, appliedVolts);
		u.set(0, 0, appliedVolts);

		// The torque on the arm is given by τ = F⋅r, where F is the force applied by
		// gravity and r the distance from pivot to center of mass. Recall from
		// dynamics that the sum of torques for a rigid body is τ = J⋅α, were τ is
		// torque on the arm, J is the mass-moment of inertia about the pivot axis,
		// and α is the angular acceleration in rad/s². Rearranging yields: α = F⋅r/J
		//
		// We substitute in F = m⋅g⋅cos(θ), where θ is the angle from horizontal:
		//
		// α = (m⋅g⋅cos(θ))⋅r/J
		//
		// Multiply RHS by cos(θ) to account for the arm angle. Further, we know the
		// arm mass-moment of inertia J of our arm is given by J=1/3 mL², modeled as a
		// rod rotating about it's end, where L is the overall rod length. The mass
		// distribution is assumed to be uniform. Substitute r=L/2 to find:
		//
		// α = (m⋅g⋅cos(θ))⋅r/(1/3 mL²)
		// α = (m⋅g⋅cos(θ))⋅(L/2)/(1/3 mL²)
		// α = 3/2⋅g⋅cos(θ)/L
		//
		// This acceleration is next added to the linear system dynamics ẋ=Ax+Bu
		//
		// f(x, u) = Ax + Bu + [0 α]ᵀ
		// f(x, u) = Ax + Bu + [0 3/2⋅g⋅cos(θ)/L]ᵀ

		Matrix<N2, N1> updatedXhat = NumericalIntegration.rkdp((Matrix<N2, N1> x, Matrix<N1, N1> _u) -> {
			Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(_u));
			if (m_simulateGravity) {
				double alphaGrav = 3.0 / 2.0 * -9.8 * Math.cos(x.get(0, 0)) / m_armLenMeters;
				xdot = xdot.plus(VecBuilder.fill(0, alphaGrav));
			}
			return xdot;
		}, currentXhat, u, dtSeconds);

		// We check for collision after updating xhat
		if (wouldHitLowerLimit(updatedXhat.get(0, 0))) { return VecBuilder.fill(m_minAngle, 0); }
		if (wouldHitUpperLimit(updatedXhat.get(0, 0))) { return VecBuilder.fill(m_maxAngle, 0); }
		return updatedXhat;
	}

	@Override
	public void simulationSubTick(int subTickNum) {
		// Step simulation by dt
		double now = Timer.getFPGATimestamp();
		if (m_lastTick == -1) update(0.02);
		else update(now - m_lastTick);
		m_lastTick = now;
	}

	/**
	 * Configures the per-motor current limits.
	 *
	 * @param supplyPerMotor The supply current limit per motor.
	 * @param statorPerMotor The stator current limit per motor.
	 */
	public void configureCurrentLimits(double supplyPerMotor, double statorPerMotor) {
		m_supplyLim = supplyPerMotor;
		m_statorLim = statorPerMotor;
	}

	// Helper: get the battery voltage in simulation or real robot
	private double getBatteryVolts() {
		if (RobotBase.isSimulation()) return SimulatedBattery.getBatteryVoltage().in(Volts);
		else return RobotController.getBatteryVoltage();
	}
}
