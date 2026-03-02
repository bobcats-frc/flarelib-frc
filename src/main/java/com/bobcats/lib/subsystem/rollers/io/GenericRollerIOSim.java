package com.bobcats.lib.subsystem.rollers.io;

import com.bobcats.lib.sim.CustomDCMotorSim;
import com.bobcats.lib.subsystem.rollers.GenericRollerIO;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import org.ironmaple.simulation.SimulatedArena;

/** The IO sim implementation for generic roller hardware interacions without any hardware. */
public class GenericRollerIOSim implements GenericRollerIO {
	/**
	 * The system standard deviations to use when simulateSystemNoise is set to true. Keep in mind
	 * that this can be modified before each instantiation of a {@link GenericRollerIOSim} to apply
	 * different standard deviations to each instance. The array defaults to <i>[0.5, 0.5,
	 * 0.2]</i>.
	 *
	 * <p>
	 * <b>Array format</b>: <i>[position (rad), velocity (rad/s), current (A)]</i>
	 */
	public static double[] kSystemStandardDeviations = new double[] { 0.5, 0.5, 0.2 };

	private CustomDCMotorSim m_sim;

	/**
	 * Constructs a new GenericRollerIOSim.
	 *
	 * @param gearbox             The motor gearbox. Do <b>not</b> use
	 *                            {@link DCMotor#withReduction(double)}.
	 * @param moi                 The MOI of the rollers, in kg*m^2.
	 * @param reduction           The reduction of each motor.
	 * @param numMotors           The number of motors driving the rollers.
	 * @param supplyCurrentLimit  The supply current limit per motor.
	 * @param statorCurrentLimit  The stator/torque current limit per motor.
	 * @param simulateSystemNoise Whether to add measurement standard deviations to the simulation.
	 */
	public GenericRollerIOSim(DCMotor gearbox, double moi, double reduction, int numMotors, double supplyCurrentLimit,
			double statorCurrentLimit, boolean simulateSystemNoise) {
		if (gearbox == null || moi <= 0.0 || reduction <= 0.0 || numMotors <= 0 || supplyCurrentLimit <= 0)
			throw new IllegalArgumentException("faulty parameters provided to GenericRollerIOSim");

		// Create sim
		m_sim = new CustomDCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, moi, reduction), gearbox, numMotors,
				simulateSystemNoise ? kSystemStandardDeviations : new double[3]);

		// Register sim
		SimulatedArena.getInstance().addCustomSimulation(m_sim);
	}

	@Override
	public void updateInputs(GenericRollerIOInputs inputs) {
		// Update output data
		inputs.motorConnected = true;
		inputs.appliedVoltage = m_sim.getAppliedInputVoltage();
		inputs.positionRevs = m_sim.getOutputAngularPositionRotations() * m_sim.getGearing();
		inputs.rpm = m_sim.getOutputAngularVelocityRPM() * m_sim.getGearing();
		inputs.statorCurrentAmps = Math.abs(m_sim.getStatorCurrentDrawAmps());
		inputs.supplyCurrentAmps = Math.abs(m_sim.getSupplyCurrentDrawAmps());
		inputs.temperatureCelsius = 20;
	}

	@Override
	public void runVolts(double volts) {
		m_sim.setInputVoltage(volts);
	}
}
