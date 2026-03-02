package com.bobcats.lib.sim;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import org.ejml.MatrixDimensionException;
import org.ejml.simple.SimpleMatrix;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.SimulatedArena.Simulatable;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

/**
 * Represents a simulated DC motor mechanism.
 *
 * <p>
 * Supports current limiting via {@link #setSupplyCurrentLimitAmps(double)} and
 * {@link #setStatorCurrentLimitAmps(double)}.
 *
 * <p>
 * To use, simply add the simulation to the {@link SimulatedArena} via
 * {@link SimulatedArena#addCustomSimulation(Simulatable)} after instantiating. Do not re-add
 * to {@link SimulatedBattery}.
 */
public class CustomDCMotorSim implements Simulatable {
	private double m_requestedVoltage = 0.0;
	private double m_appliedVoltage = 0.0;

	private double m_statorCurrentLimitAmps = Double.POSITIVE_INFINITY;
	private double m_supplyCurrentLimitAmps = Double.POSITIVE_INFINITY;

	private final int m_numMotors;

	private double m_lastTick = -1;

	// To get accurate data in this class, we artificially add noise to the getters
	private Matrix<N3, N1> m_measurementStdDevs;
	private Matrix<N3, N1> m_whiteNoiseVector;

	// Gearbox for the DC motor.
	private final DCMotor m_gearbox;

	// The gearing from the motors to the output.
	private double m_gearing;

	// The moment of inertia for the DC motor mechanism.
	private double m_jKgMetersSquared;

	private LinearSystem<N2, N1, N2> m_plant;

	/** State vector. */
	private Matrix<N2, N1> m_x;

	/** Input vector. */
	private Matrix<N1, N1> m_u;

	/** Output vector. */
	private Matrix<N2, N1> m_y;

	/**
	 * Constructs a new CustomDCMotorSim.
	 *
	 * @param plant              The plant model of the DC motor system.
	 * @param gearbox            The gearbox model of the DC motor system.
	 * @param numMotors          The number of motors in the system.
	 * @param measurementStdDevs The standard deviations of the measurements. Can be omitted if no
	 *                           noise is desired. If present must have 3 elements. The first
	 *                           element is for position. The second element is for velocity. The
	 *                           third element is for current (used for both supply and stator).
	 *                           Format: [x, w, I].
	 */
	public CustomDCMotorSim(LinearSystem<N2, N1, N2> plant, DCMotor gearbox, int numMotors,
			double... measurementStdDevs) {
		// super(plant, gearbox);
		if (measurementStdDevs.length != 0 && measurementStdDevs.length != 3) {
			throw new MatrixDimensionException(
					"Malformed measurementStdDevs! Got " + measurementStdDevs.length + " elements instead of 3");
		}

		m_numMotors = numMotors;
		SimulatedBattery.addElectricalAppliances(() -> Amps.of(Math.abs(getSupplyCurrentDrawAmps_noNoise())));

		if (measurementStdDevs.length == 0) {
			m_measurementStdDevs = new Matrix<>(new SimpleMatrix(3, 1));
		} else {
			m_measurementStdDevs = new Matrix<>(new SimpleMatrix(measurementStdDevs));
		}
		m_whiteNoiseVector = StateSpaceUtil.makeWhiteNoiseVector(m_measurementStdDevs);

		m_gearbox = gearbox;

		m_plant = plant;

		m_x = new Matrix<>(new SimpleMatrix(plant.getA().getNumRows(), 1));
		m_u = new Matrix<>(new SimpleMatrix(plant.getB().getNumCols(), 1));
		m_y = new Matrix<>(new SimpleMatrix(plant.getC().getNumRows(), 1));

		// Credits WPILib:
		// By theorem 6.10.1 of https://file.tavsys.net/control/controls-engineering-in-frc.pdf,
		// the DC motor state-space model is:
		//
		// dx/dt = -G²Kₜ/(KᵥRJ)x + (GKₜ)/(RJ)u
		// A = -G²Kₜ/(KᵥRJ)
		// B = GKₜ/(RJ)
		//
		// Solve for G.
		//
		// A/B = -G/Kᵥ
		// G = -KᵥA/B
		//
		// Solve for J.
		//
		// B = GKₜ/(RJ)
		// J = GKₜ/(RB)
		m_gearing = -gearbox.KvRadPerSecPerVolt * plant.getA(1, 1) / plant.getB(1, 0);
		m_jKgMetersSquared = m_gearing * gearbox.KtNMPerAmp / (gearbox.rOhms * plant.getB(1, 0));
	}

	/**
	 * Sets the per-motor stator current limit. Set to {@link Double#POSITIVE_INFINITY} to disable.
	 *
	 * @param amps The limit, in Amps.
	 */
	public void setStatorCurrentLimitAmps(double amps) {
		if (amps <= 0 && !Double.isInfinite(amps))
			throw new IllegalArgumentException("stator limit must be > 0 or +Inf to disable");
		m_statorCurrentLimitAmps = amps * m_numMotors;
	}

	/**
	 * Sets the per-motor supply current limit. Set to {@link Double#POSITIVE_INFINITY} to disable.
	 *
	 * @param amps The limit, in Amps.
	 */
	public void setSupplyCurrentLimitAmps(double amps) {
		if (amps <= 0 && !Double.isInfinite(amps))
			throw new IllegalArgumentException("supply limit must be > 0 or +Inf to disable");
		m_supplyCurrentLimitAmps = amps * m_numMotors;
	}

	/**
	 * Returns the requested motor input voltage via {@link #setInputVoltage(double)}, before any
	 * current limtits.
	 *
	 * @return The requested input voltage, in Volts.
	 */
	public double getRequestedInputVoltage() { return m_requestedVoltage; }

	/**
	 * Returns the applied motor input voltage after any current limtits.
	 *
	 * @return The applied voltage, in Volts.
	 */
	public double getAppliedInputVoltage() { return m_appliedVoltage; }

	/**
	 * Sets the requested voltage of the DC motor.
	 *
	 * @param volts The requested voltage.
	 */
	public void setInputVoltage(double volts) { m_requestedVoltage = volts; }

	/**
	 * Sets the plant. Note that the gearbox must be the same.
	 *
	 * @param newPlant The new plant.
	 */
	protected void setPlant(LinearSystem<N2, N1, N2> newPlant) {
		m_plant = newPlant;

		m_gearing = -m_gearbox.KvRadPerSecPerVolt * newPlant.getA(1, 1) / newPlant.getB(1, 0);
		m_jKgMetersSquared = m_gearing * m_gearbox.KtNMPerAmp / (m_gearbox.rOhms * newPlant.getB(1, 0));
	}

	/**
	 * Updates the state of the DC motor.
	 *
	 * @param currentXhat The current state estimate.
	 * @param u           The system inputs (voltage).
	 * @param dtSeconds   The time difference between controller updates.
	 * @return The updated state.
	 */
	protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
		// Motor shaft speed in rad/s, W = W_out * G
		double omegaMotor = currentXhat.get(1, 0) * getGearing();

		// Desaturate voltage to stay within current limits
		double appliedVolts = HardwareSimUtils.desaturateVoltage(m_gearbox, omegaMotor, getBatteryVolts(),
				m_requestedVoltage, m_appliedVoltage, m_statorCurrentLimitAmps, m_supplyCurrentLimitAmps);

		// Set input to the chosen (limited) voltage and integrate (applied voltage)
		m_appliedVoltage = appliedVolts;
		// setInputVoltage(appliedVolts);
		u.set(0, 0, appliedVolts);

		m_whiteNoiseVector = StateSpaceUtil.makeWhiteNoiseVector(m_measurementStdDevs);

		// Call plant updateX to compute the next state
		return m_plant.calculateX(currentXhat, u, dtSeconds);
	}

	/**
	 * Updates the simulation.
	 *
	 * @param dtSeconds The time between updates.
	 */
	protected void update(double dtSeconds) {
		// Update X. By default, this is the linear system dynamics X = Ax + Bu
		m_x = updateX(m_x, m_u, dtSeconds);

		// y = cx + du
		m_y = m_plant.calculateY(m_x, m_u);
	}

	/**
	 * Sets the state of the DC motor.
	 *
	 * @param angularPositionRad       The new position in radians.
	 * @param angularVelocityRadPerSec The new velocity in radians per second.
	 */
	public void setState(double angularPositionRad, double angularVelocityRadPerSec) {
		setState(VecBuilder.fill(angularPositionRad, angularVelocityRadPerSec));
	}

	/**
	 * Sets the system state.
	 *
	 * @param state The new state.
	 */
	protected void setState(Matrix<N2, N1> state) { m_x = state; }

	/**
	 * Sets the DC motor's angular position.
	 *
	 * @param angularPositionRad The new position in radians.
	 */
	public void setOutputAngle(double angularPositionRad) {
		setState(angularPositionRad, getOutputAngularVelocityRadPerSec_noNoise());
	}

	/**
	 * Sets the DC motor's angular velocity.
	 *
	 * @param angularVelocityRadPerSec The new velocity in radians per second.
	 */
	public void setOutputAngularVelocity(double angularVelocityRadPerSec) {
		setState(getOutputAngularPositionRad_noNoise(), angularVelocityRadPerSec);
	}

	/**
	 * Returns the gear ratio of the DC motor.
	 *
	 * @return the DC motor's gear ratio.
	 */
	public double getGearing() { return m_gearing; }

	/**
	 * Returns the moment of inertia of the DC motor.
	 *
	 * @return The DC motor's moment of inertia.
	 */
	public double getJKgMetersSquared() { return m_jKgMetersSquared; }

	/**
	 * Returns the gearbox for the DC motor.
	 *
	 * @return The DC motor's gearbox.
	 */
	public DCMotor getGearbox() { return m_gearbox; }

	/**
	 * Returns the system's total supply current draw.
	 *
	 * @return The total supply current draw, in Amps.
	 */
	public double getSupplyCurrentDrawAmps() {
		return getCurrentDrawAmps_noNoise() * getAppliedInputVoltage() / getBatteryVolts()
				+ m_whiteNoiseVector.get(2, 0);
	}

	/**
	 * Returns the system's total stator current draw.
	 *
	 * @return The total stator current draw, in Amps.
	 */
	public double getStatorCurrentDrawAmps() { return getCurrentDrawAmps_noNoise() + m_whiteNoiseVector.get(2, 0); }

	/**
	 * Returns the DC motor's position.
	 *
	 * @return The DC motor's position.
	 */
	public double getOutputAngularPositionRad() { return getOutput(0) + m_whiteNoiseVector.get(0, 0); }

	/**
	 * Returns the DC motor's position in rotations.
	 *
	 * @return The DC motor's position in rotations.
	 */
	public double getOutputAngularPositionRotations() {
		return Units.radiansToRotations(getOutputAngularPositionRad());
	}

	/**
	 * Returns the DC motor's position.
	 *
	 * @return The DC motor's position
	 */
	public Angle getOutputAngularPosition() { return Radians.of(getOutputAngularPositionRad()); }

	/**
	 * Returns the DC motor's velocity.
	 *
	 * @return The DC motor's velocity.
	 */
	public double getOutputAngularVelocityRadPerSec() { return getOutput(1) + m_whiteNoiseVector.get(1, 0); }

	/**
	 * Returns the DC motor's velocity in RPM.
	 *
	 * @return The DC motor's velocity in RPM.
	 */
	public double getOutputAngularVelocityRPM() {
		return Units.radiansPerSecondToRotationsPerMinute(getOutputAngularVelocityRadPerSec());
	}

	/**
	 * Returns the DC motor's acceleration in rad/s^2.
	 *
	 * @return The DC motor's acceleration in rad/s^2.
	 */
	public double getOutputAngularAccelerationRadPerSecSq() {
		var acceleration = (m_plant.getA().times(m_x)).plus(m_plant.getB().times(m_u));
		return acceleration.get(1, 0);
	}

	/**
	 * Returns the DC motor's acceleration.
	 *
	 * @return The DC motor's acceleration.
	 */
	public AngularAcceleration getOutputAngularAcceleration() {
		return RadiansPerSecondPerSecond.of(getOutputAngularAccelerationRadPerSecSq());
	}

	/**
	 * Returns the DC motor's torque in Newton-Meters.
	 *
	 * @return The DC motor's torque in Newton-Meters.
	 */
	public double getTorqueNewtonMeters() { return getOutputAngularAccelerationRadPerSecSq() * m_jKgMetersSquared; }

	/**
	 * Returns the DC motor's velocity.
	 *
	 * @return The DC motor's velocity
	 */
	public AngularVelocity getOutputAngularVelocity() {
		return RadiansPerSecond.of(getOutputAngularVelocityRadPerSec());
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

	// Maplesim interaction
	@Override
	public void simulationSubTick(int subTickNum) {
		// Step simulation by dt
		double now = Timer.getFPGATimestamp();
		if (m_lastTick == -1) update(0.02);
		else update(now - m_lastTick);
		m_lastTick = now;
	}

	// Helper: get the battery voltage in simulation or real robot
	private double getBatteryVolts() {
		if (RobotBase.isSimulation()) return SimulatedBattery.getBatteryVoltage().in(Volts);
		else return RobotController.getBatteryVoltage();
	}

	// Helper: current for a given voltage at given output speed (rad/s)
	private double getCurrentAt(double volts, double outputSpeedRadPerSec) {
		return getGearbox().getCurrent(outputSpeedRadPerSec, volts);
	}

	// No-noise methods

	private double getCurrentDrawAmps_noNoise() {
		double omegaOut = getOutputAngularVelocityRadPerSec_noNoise() * getGearing();
		return getCurrentAt(m_appliedVoltage, omegaOut);
	}

	private double getOutputAngularVelocityRadPerSec_noNoise() { return getOutput(1); }

	private double getOutputAngularPositionRad_noNoise() { return getOutput(0); }

	private double getSupplyCurrentDrawAmps_noNoise() {
		return getCurrentDrawAmps_noNoise() * getAppliedInputVoltage() / getBatteryVolts();
	}
}
