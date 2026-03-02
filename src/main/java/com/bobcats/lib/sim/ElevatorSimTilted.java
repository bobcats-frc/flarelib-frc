// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//
// NOTE:
// Taken from the WPILib Java library. Most credit goes to them.
//

package com.bobcats.lib.sim;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Volts;

import com.bobcats.lib.control.SubsystemRatios;
import com.bobcats.lib.utils.Utils;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.ejml.MatrixDimensionException;
import org.ejml.simple.SimpleMatrix;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.SimulatedArena.Simulatable;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

/**
 * Represents a simulated elevator mechanism. Allows for the use of non-90 degree tilted
 * systems.
 *
 * <p>
 * This is a modified version of WPILib's {@link ElevatorSim}, simplifying the calculations on
 * the user-end, adding angled elevator support, and integrating with MapleSim's
 * {@link SimulatedBattery}.
 *
 * <p>
 * Supports current limiting via {@link #setSupplyCurrentLimitAmps(double)} and
 * {@link #setStatorCurrentLimitAmps(double)}.
 *
 * <p>
 * To use, simply add the simulation to the {@link SimulatedArena} via
 * {@link SimulatedArena#addCustomSimulation(Simulatable)} after instantiating.
 */
public class ElevatorSimTilted implements Simulatable {

	// Gearbox for the elevator.
	private final DCMotor m_gearbox;

	// The min allowable height for the elevator.
	private final double m_minHeight;

	// The max allowable height for the elevator.
	private final double m_maxHeight;

	// Whether the simulator should simulate gravity.
	private final boolean m_simulateGravity;

	// The angle at which the system is positioned.
	private double m_tilt = 0;

	/** The plant that represents the linear system. */
	private LinearSystem<N2, N1, N2> m_plant;

	/** State vector. */
	private Matrix<N2, N1> m_x;

	/** Input vector. */
	private Matrix<N1, N1> m_u;

	/** Output vector. */
	private Matrix<N2, N1> m_y;

	/** The standard deviations of measurements, used for adding noise to the measurements. */
	private final Matrix<N3, N1> m_measurementStdDevs;
	private Matrix<N3, N1> m_noiseMatrix;

	// Elevator data
	private final boolean m_isCascade;
	private final double m_radius, m_gearing;
	private final double[] m_masses, m_extensions;
	private double m_effMass = 0;
	private double m_lastEffMass = 0;

	private double m_supplyLimit = Double.POSITIVE_INFINITY;
	private double m_statorLimit = Double.POSITIVE_INFINITY;
	private final int m_numMotors;

	private double m_lastTick = -1;

	private double m_lastCurrent, m_lastCurrentSupply;

	private double m_requestedVolts;

	private double m_metersPerRev;

	/**
	 * The gravitational acceleration of the earth, in m/s^2. This value should always be positive.
	 */
	public double G = 9.80665;

	/**
	 * Creates a simulated continuous elevator mechanism.
	 *
	 * @param gearbox               The type of and number of motors in the elevator gearbox.
	 * @param gearing               The motor reduction.
	 * @param stageMasses           The mass of each stage in kilograms, in order.
	 * @param stageExtensionHeights The height at which each stage starts extending, in meters.
	 * @param drumRadiusMeters      The radius of the driving drum, in meters.
	 * @param minHeightMeters       The min allowable height of the elevator.
	 * @param maxHeightMeters       The max allowable height of the elevator.
	 * @param simulateGravity       Whether gravity should be simulated or not.
	 * @param startingHeightMeters  The starting height of the elevator.
	 * @param numMotors             The number of motors driving the elevator.
	 * @param measurementStdDevs    The standard deviations of the measurements. Can be omitted if
	 *                              no noise is desired. If present must have 3 elements for
	 *                              position, velocity and current. Units are in meters, m/s and
	 *                              Amps respectively.
	 */
	public ElevatorSimTilted(DCMotor gearbox, double gearing, double[] stageMasses, double[] stageExtensionHeights,
			double drumRadiusMeters, double minHeightMeters, double maxHeightMeters, boolean simulateGravity,
			double startingHeightMeters, int numMotors, double... measurementStdDevs) {
		if (stageMasses.length == 0 || stageMasses.length != stageExtensionHeights.length)
			throw new IllegalArgumentException("Invalid stage amount, or mass and extensions don't match");
		var plant = LinearSystemId.createElevatorSystem(gearbox, stageMasses[0], drumRadiusMeters, gearing);
		if (measurementStdDevs.length != 0 && measurementStdDevs.length != 3) throw new MatrixDimensionException(
				"Malformed measurementStdDevs! Got " + measurementStdDevs.length + " elements instead of 3");
		m_plant = plant;
		m_radius = drumRadiusMeters;
		m_gearing = gearing;
		m_masses = stageMasses;
		m_extensions = stageExtensionHeights;
		m_isCascade = false;

		// Create matrices
		if (measurementStdDevs.length == 0) {
			m_measurementStdDevs = new Matrix<>(new SimpleMatrix(3, 1));
		} else {
			m_measurementStdDevs = new Matrix<>(new SimpleMatrix(measurementStdDevs));
		}
		m_x = new Matrix<>(new SimpleMatrix(plant.getA().getNumRows(), 1));
		m_u = new Matrix<>(new SimpleMatrix(plant.getB().getNumCols(), 1));
		m_y = new Matrix<>(new SimpleMatrix(plant.getC().getNumRows(), 1));
		m_noiseMatrix = StateSpaceUtil.makeWhiteNoiseVector(m_measurementStdDevs);

		m_gearbox = gearbox;
		m_minHeight = minHeightMeters;
		m_maxHeight = maxHeightMeters;
		m_simulateGravity = simulateGravity;

		m_numMotors = numMotors;

		m_metersPerRev = SubsystemRatios.elevator_calculateSTMR(drumRadiusMeters, gearing);

		SimulatedBattery.addElectricalAppliances(() -> Amps.of(Math.abs(getSupplyCurrentDrawAmps())));
		setState(startingHeightMeters, 0);
	}

	/**
	 * Creates a simulated cascade elevator mechanism.
	 *
	 * @param gearbox              The type of and number of motors in the elevator gearbox.
	 * @param gearing              The motor reduction.
	 * @param stageMasses          The mass of each stage in kilograms, in order.
	 * @param drumRadiusMeters     The radius of the driving drum, in meters.
	 * @param minHeightMeters      The min allowable height of the elevator.
	 * @param maxHeightMeters      The max allowable height of the elevator.
	 * @param simulateGravity      Whether gravity should be simulated or not.
	 * @param startingHeightMeters The starting height of the elevator.
	 * @param numMotors            The number of motors driving the elevator.
	 * @param measurementStdDevs   The standard deviations of the measurements. Can be omitted if
	 *                             no noise is desired. If present must have 3 elements for
	 *                             position, velocity and current. Units are in meters, m/s and
	 *                             Amps respectively.
	 */
	public ElevatorSimTilted(DCMotor gearbox, double gearing, double[] stageMasses, double drumRadiusMeters,
			double minHeightMeters, double maxHeightMeters, boolean simulateGravity, double startingHeightMeters,
			int numMotors, double... measurementStdDevs) {
		if (stageMasses.length == 0) throw new IllegalArgumentException("Invalid stage amount for cascade");
		var plant = LinearSystemId.createElevatorSystem(gearbox, stageMasses[0], drumRadiusMeters, gearing);
		if (measurementStdDevs.length != 0 && measurementStdDevs.length != 3) throw new MatrixDimensionException(
				"Malformed measurementStdDevs! Got " + measurementStdDevs.length + " elements instead of 3");
		m_plant = plant;
		m_radius = drumRadiusMeters;
		m_gearing = gearing;
		m_masses = stageMasses;
		m_extensions = null;
		m_isCascade = true;

		// Effective mass for a cascade
		for (int i = 0; i < stageMasses.length; i++) { m_effMass += Math.pow(i + 1, 2) * stageMasses[i]; }

		// Create matrices
		if (measurementStdDevs.length == 0) {
			m_measurementStdDevs = new Matrix<>(new SimpleMatrix(3, 1));
		} else {
			m_measurementStdDevs = new Matrix<>(new SimpleMatrix(measurementStdDevs));
		}
		m_x = new Matrix<>(new SimpleMatrix(plant.getA().getNumRows(), 1));
		m_u = new Matrix<>(new SimpleMatrix(plant.getB().getNumCols(), 1));
		m_y = new Matrix<>(new SimpleMatrix(plant.getC().getNumRows(), 1));
		m_noiseMatrix = StateSpaceUtil.makeWhiteNoiseVector(m_measurementStdDevs);

		m_gearbox = gearbox;
		m_minHeight = minHeightMeters;
		m_maxHeight = maxHeightMeters;
		m_simulateGravity = simulateGravity;
		m_numMotors = numMotors;

		m_metersPerRev = SubsystemRatios.elevator_calculateSTMR(drumRadiusMeters, gearing);

		SimulatedBattery.addElectricalAppliances(() -> Amps.of(Math.abs(getSupplyCurrentDrawAmps())));
		setState(startingHeightMeters, 0);
	}

	/**
	 * Sets the system's tilt (offset from the normal) to the given angle, defaults to 0 degrees
	 * for upright.
	 *
	 * @param angle The tilt angle, in degrees.
	 */
	public void setSystemTilt(double angle) { m_tilt = Math.toRadians(angle); }

	/**
	 * Returns whether the elevator would hit the lower limit.
	 *
	 * @param elevatorHeightMeters The elevator height.
	 * @return Whether the elevator would hit the lower limit.
	 */
	public boolean wouldHitLowerLimit(double elevatorHeightMeters) {
		return elevatorHeightMeters <= m_minHeight;
	}

	/**
	 * Returns whether the elevator would hit the upper limit.
	 *
	 * @param elevatorHeightMeters The elevator height.
	 * @return Whether the elevator would hit the upper limit.
	 */
	public boolean wouldHitUpperLimit(double elevatorHeightMeters) {
		return elevatorHeightMeters >= m_maxHeight;
	}

	/**
	 * Returns whether the elevator has hit the lower limit.
	 *
	 * @return Whether the elevator has hit the lower limit.
	 */
	public boolean hasHitLowerLimit() {
		return wouldHitLowerLimit(getPositionMeters());
	}

	/**
	 * Returns whether the elevator has hit the upper limit.
	 *
	 * @return Whether the elevator has hit the upper limit.
	 */
	public boolean hasHitUpperLimit() {
		return wouldHitUpperLimit(getPositionMeters());
	}

	/**
	 * Returns the position of the elevator.
	 *
	 * @return The position of the elevator.
	 */
	public double getPositionMeters() { return getOutput(0) + m_noiseMatrix.get(0, 0); }

	/**
	 * Returns the velocity of the elevator.
	 *
	 * @return The velocity of the elevator.
	 */
	public double getVelocityMetersPerSecond() { return getOutput(1) + m_noiseMatrix.get(1, 0); }

	/**
	 * Returns the elevator current draw. Returns the stator current, see
	 * {@link #getSupplyCurrentDrawAmps()} for the supply current.
	 *
	 * @return The elevator current draw.
	 */
	public double getCurrentDrawAmps() { return m_lastCurrent + m_noiseMatrix.get(2, 0); }

	/**
	 * Returns the elevator current draw. Returns the supply current, see
	 * {@link #getCurrentDrawAmps()} for the stator current.
	 *
	 * @return The elevator current draw.
	 */
	public double getSupplyCurrentDrawAmps() { return m_lastCurrentSupply + m_noiseMatrix.get(2, 0); }

	/**
	 * Sets the requested voltage of the elevator.
	 *
	 * @param volts The requested voltage.
	 */
	public void setInputVoltage(double volts) { m_requestedVolts = volts; }

	/**
	 * Updates the state of the elevator.
	 *
	 * @param currentXhat The current state estimate.
	 * @param u           The system inputs (voltage).
	 * @param dtSeconds   The time difference between controller updates.
	 * @return The updated state.
	 */
	protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
		if (G <= 0) DriverStation.reportWarning(
				"WARNING: ElevatorSimTilted::updateX, gravitational acceleration G <= 0, this is likely a mistake",
				false);
		// Compute single-stage effective mass
		double effectiveMass = 0;
		if (!m_isCascade) for (int i = 0; i < m_extensions.length; i++) {
			if (m_extensions[i] <= currentXhat.get(0, 0)) effectiveMass += m_masses[i];
		}
		else effectiveMass = m_effMass;
		m_effMass = effectiveMass;

		// Invalid mass
		if (m_effMass == 0) {
			DriverStation.reportWarning(
					"WARNING: ElevatorSimTilted::updateX, an effective mass of 0 was given, this is likely a bug",
					false);
			return currentXhat;
		}

		// Update plant if nescessary
		if (!Utils.epsilonEquals(effectiveMass, m_lastEffMass))
			m_plant = LinearSystemId.createElevatorSystem(m_gearbox, effectiveMass, m_radius, m_gearing);
		m_lastEffMass = effectiveMass;

		// // Calculate updated x-hat from Runge-Kutta.
		// var updatedXhat = NumericalIntegration.rkdp((x, _u) -> {
		// Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(_u));
		// if (m_simulateGravity) xdot = xdot.plus(VecBuilder.fill(0, -G * Math.cos(m_tilt)));
		// return xdot;
		// }, currentXhat, u, dtSeconds);

		// var updatedXhat = NumericalIntegration.rk4((x, _u) -> {
		// Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(_u));
		// if (m_simulateGravity) { xdot = xdot.plus(VecBuilder.fill(0, -G * Math.cos(m_tilt))); }
		// return xdot;
		// }, currentXhat, u, dtSeconds);

		var updatedXhat = NumericalIntegration.rkdp((x, _u) -> {
			var xdot = m_plant.getA().times(x).plus(m_plant.getB().times(_u));
			if (m_simulateGravity) xdot = xdot.plus(VecBuilder.fill(0, -G * Math.cos(m_tilt)));
			return xdot;
		}, m_x, m_u, dtSeconds);

		// We check for collisions after updating x-hat.
		if (wouldHitLowerLimit(updatedXhat.get(0, 0))) return VecBuilder.fill(m_minHeight, 0);
		if (wouldHitUpperLimit(updatedXhat.get(0, 0))) return VecBuilder.fill(m_maxHeight, 0);
		return updatedXhat;
	}

	/**
	 * Updates the simulation.
	 *
	 * @param dtSeconds The time between updates.
	 */
	protected void update(double dtSeconds) {
		// Motor shaft speed in rad/s, w = 2pi * v / f
		double omegaOutput = Units.rotationsToRadians(m_x.get(1, 0) / m_metersPerRev);

		// Desaturate voltage to stay within current limits
		double appliedVolts = HardwareSimUtils.desaturateVoltage(m_gearbox, omegaOutput, getBatteryVolts(),
				m_requestedVolts, getInput(0), m_statorLimit, m_supplyLimit);

		// Update inputs
		m_u.set(0, 0, appliedVolts);

		// Update X. By default, this is the linear system dynamics X = Ax + Bu
		m_x = updateX(m_x, m_u, dtSeconds);

		// y = cx + du
		m_y = m_plant.calculateY(m_x, m_u);

		// Create noise matrix
		m_noiseMatrix = StateSpaceUtil.makeWhiteNoiseVector(m_measurementStdDevs);
	}

	/**
	 * Returns the current output of the plant.
	 *
	 * @return The current output of the plant.
	 */
	protected Matrix<N2, N1> getOutput() { return m_y; }

	/**
	 * Returns an element of the current output of the plant.
	 *
	 * @param row The row to return.
	 * @return An element of the current output of the plant.
	 */
	protected double getOutput(int row) {
		return m_y.get(row, 0);
	}

	/**
	 * Returns an element of the current input of the plant.
	 *
	 * @param row The row to return.
	 * @return An element of the current input of the plant.
	 */
	protected double getInput(int row) {
		return m_u.get(row, 0);
	}

	/**
	 * Returns the post-limits applied input voltage.
	 *
	 * @return The applied input voltage.
	 */
	public double getAppliedInputVoltage() { return getInput(0); }

	/**
	 * Returns the pre-limits requested input voltage.
	 *
	 * @return The requested input voltage.
	 */
	public double getRequestedInputVoltage() { return m_requestedVolts; }

	/**
	 * Sets the elevator's state. The new position will be limited between the minimum and maximum
	 * allowed heights. Useful for resetting position and/or velocity. Parameters must be position
	 * and velocity with the units meters and m/s respectively.
	 *
	 * @param positionMeters          The new position in meters.
	 * @param velocityMetersPerSecond New velocity in meters per second.
	 */
	public void setState(double positionMeters, double velocityMetersPerSecond) {
		setState(VecBuilder.fill(MathUtil.clamp(positionMeters, m_minHeight, m_maxHeight), velocityMetersPerSecond));
	}

	/**
	 * Sets the system state. Useful for resetting position and/or velocity. Matrix must be: [x,
	 * v], in meters and m/s respectively.
	 *
	 * @param state The new state.
	 */
	protected void setState(Matrix<N2, N1> state) { m_x = state; }

	/**
	 * Sets the maximum supply current of each motor. By default, the limits are disabled.
	 *
	 * @param maxSupply The supply current limit, in Amps.
	 */
	public void setSupplyCurrentLimitAmps(double maxSupply) {
		if (maxSupply < 0)
			throw new IllegalArgumentException("supply current limit must be non-negative, or +Inf to disable");
		m_supplyLimit = maxSupply * m_numMotors;
	}

	/**
	 * Sets the maximum stator current of each motor. By default, the limits are disabled.
	 *
	 * @param maxStator The stator current limit, in Amps.
	 */
	public void setStatorCurrentLimitAmps(double maxStator) {
		if (maxStator < 0)
			throw new IllegalArgumentException("stator current limit must be non-negative, or +Inf to disable");
		m_statorLimit = maxStator * m_numMotors;
	}

	/**
	 * Returns the single-carriage effective mass of the elevator, where the net structural mass of
	 * the elevator is represented by the mass of a 1-stage singular carriage. This depends on the
	 * stage masses, the stage extensions, whether the elevator is cascading or continuous, and the
	 * current elevator height. This value is constant for cascade elevators, but is variable for
	 * continuous elevators.
	 *
	 * <p>
	 * For cascade: {@code sum(i, 1->n, m_i * i^2)}
	 *
	 * <p>
	 * For continuous: {@code sum(i, 1->max extended/extending stage, m_i)}
	 *
	 * @return The effective mass, in kilograms.
	 */
	public double getEffectiveMassKg() { return m_effMass; }

	/**
	 * Returns the single-carriage effective mass of the elevator, where the net structural mass of
	 * the elevator is represented by the mass of a 1-stage singular carriage. This depends on the
	 * stage masses, the stage extensions, whether the elevator is cascading or continuous, and the
	 * current elevator height. This value is constant for cascade elevators, but is variable for
	 * continuous elevators.
	 *
	 * <p>
	 * For cascade: {@code sum(i, 1->n, m_i * i^2)}
	 *
	 * <p>
	 * For continuous: {@code sum(i, 1->max extended/extending stage, m_i)}
	 *
	 * @return The effective mass, represented by Mass.
	 */
	public Mass getEffectiveMass() { return Kilograms.of(getEffectiveMassKg()); }

	@Override
	public void simulationSubTick(int subTickNum) {
		// Step simulation by dt
		double now = Timer.getFPGATimestamp();
		if (m_lastTick == -1) update(0.02);
		else update(now - m_lastTick);
		m_lastTick = now;

		// Calculations moved to the update method to fix desync at different
		// frequency loops, and to avoid noise
		// (e.g. 50Hz robot loop vs 250Hz MapleSim loop)

		// Taken from WPILib:
		// I = V / R - omega / (Kv * R)
		// Reductions are greater than 1, so a reduction of 10:1 would mean the motor is
		// spinning 10x faster than the output
		// v = r w, so w = v/r
		double kA = 1 / m_plant.getB().get(1, 0);
		double kV = -m_plant.getA().get(1, 1) * kA;
		double motorVelocityRadPerSec = m_x.get(1, 0) * kV * m_gearbox.KvRadPerSecPerVolt;
		var appliedVoltage = m_u.get(0, 0);
		m_lastCurrent = m_gearbox.getCurrent(motorVelocityRadPerSec, appliedVoltage); // * Math.signum(appliedVoltage);

		// I_supply = I_stator * D, where D is the duty cycle V_app / V_in
		m_lastCurrentSupply = m_lastCurrent * getAppliedInputVoltage() / getBatteryVolts();
	}

	// Private Methods //

	// Helper: get the battery voltage in simulation or real robot
	private double getBatteryVolts() {
		if (RobotBase.isSimulation()) return SimulatedBattery.getBatteryVoltage().in(Volts);
		else return RobotController.getBatteryVoltage();
	}
}
