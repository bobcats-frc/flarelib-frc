package com.bobcats.lib.subsystem.bangbangElevator.io;

import com.bobcats.lib.control.SubsystemRatios;
import com.bobcats.lib.sim.ElevatorSimTilted;
import com.bobcats.lib.subsystem.bangbangElevator.BangBangElevatorIO;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.SimulatedArena;

/**
 * The IO sim implementation for bang-bang elevator hardware interacions without any hardware.
 */
public class BangBangElevatorIOSim implements BangBangElevatorIO {

	/**
	 * The system standard deviations to use when simulateSystemNoise is set to true. Keep in mind
	 * that this can be modified before each instantiation of a {@link BangBangElevatorIOSim} to
	 * apply different standard deviations to each instance. The array defaults to <i>[0.01, 0.02,
	 * 0.2]</i>.
	 *
	 * <p>
	 * <b>Array format</b>: <i>[position (m), velocity (m/s), current (A)]</i>
	 */
	public static double[] kNoiseStdDevs = new double[] { 0.01, 0.02, 0.2 };

	private ElevatorSimTilted m_sim;

	private DCMotor m_gearbox;
	private double m_metersPerRev;
	private int m_numMotors;

	private boolean m_isTorqueCurrent;
	private double m_torqueCurrent;

	/**
	 * Constructs a new BangBangElevatorIOSim.
	 *
	 * @param gearbox             The gearbox driving the elevator.
	 * @param reduction           The reduction of the motors.
	 * @param numMotors           The number of motors driving the elevator.
	 * @param carriageMassKg      The mass being carried by the elevator, in kilograms.
	 * @param drumRadiusMeters    The radius of the drum, in meters.
	 * @param minHeightMeters     The minimum height of the elevator, in meters.
	 * @param maxHeightMeters     The maximum height of the elevator, in meters.
	 * @param tiltDegrees         The tilt of the elevator from the normal axis, in degrees. 0
	 *                            means perfectly upright.
	 * @param supplyCurrentLimit  The per-motor supply current limit, in Amps.
	 * @param statorCurrentLimit  The per-motor stator current limit, in Amps.
	 * @param simulateSystemNoise Whether to add simulated noise to the system.
	 */
	public BangBangElevatorIOSim(DCMotor gearbox, double reduction, int numMotors, double carriageMassKg,
			double drumRadiusMeters, double minHeightMeters, double maxHeightMeters, double tiltDegrees,
			double supplyCurrentLimit, double statorCurrentLimit, boolean simulateSystemNoise) {
		m_sim = new ElevatorSimTilted(gearbox, reduction, new double[] { carriageMassKg }, drumRadiusMeters,
				minHeightMeters, maxHeightMeters, true, 0, numMotors,
				simulateSystemNoise ? kNoiseStdDevs : new double[3]);
		m_sim.setSupplyCurrentLimitAmps(supplyCurrentLimit);
		m_sim.setStatorCurrentLimitAmps(statorCurrentLimit);
		m_sim.setSystemTilt(tiltDegrees);

		m_numMotors = numMotors;
		m_gearbox = gearbox;
		m_metersPerRev = SubsystemRatios.elevator_calculateSTMR(drumRadiusMeters, reduction);

		// Register sim
		SimulatedArena.getInstance().addCustomSimulation(m_sim);
	}

	@Override
	public void updateInputs(BangBangElevatorIOInputs inputs) {
		double rpm = m_sim.getVelocityMetersPerSecond() * 60.0 / m_metersPerRev;

		if (m_isTorqueCurrent) {
			// V = IR + w/kV
			// R_per = R * N
			m_sim.setInputVoltage(m_torqueCurrent * (m_gearbox.rOhms * m_numMotors)
					+ Units.rotationsPerMinuteToRadiansPerSecond(rpm) / m_gearbox.KvRadPerSecPerVolt);
		}

		inputs.motorConnected = true;
		inputs.appliedVoltage = m_sim.getAppliedInputVoltage();
		inputs.positionRevs = m_sim.getPositionMeters() / m_metersPerRev;
		inputs.velocityRPM = rpm;
		inputs.supplyCurrentAmps = Math.abs(m_sim.getSupplyCurrentDrawAmps());
		inputs.statorCurrentAmps = Math.abs(m_sim.getCurrentDrawAmps());
		inputs.temperatureCelsius = 20;
	}

	@Override
	public void runVolts(double volts) {
		m_isTorqueCurrent = false;
		m_sim.setInputVoltage(volts);
	}

	@Override
	public void runTorqueCurrent(double amps) {
		m_isTorqueCurrent = true;
		m_torqueCurrent = amps;
	}

	/**
	 * Returns the simulated elevator mechanism.
	 *
	 * @return The simulated elevator mechanism.
	 */
	public ElevatorSimTilted getSim() { return m_sim; }
}
