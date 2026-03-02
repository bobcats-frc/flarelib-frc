package com.bobcats.lib.subsystem.rollers;

import com.bobcats.lib.subsystem.rollers.GenericRollerSubsystem.GenericRollerVoltage;
import com.bobcats.lib.utils.Tracer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

// Inspired by Team 6328

/**
 * A generic roller subsystem, used for intakes, dispensers - rollers that don't need precise
 * velocity control.
 *
 * <p>
 * <b>Note</b>: Make sure to call super.periodic() in every periodic loop.
 *
 * <p>
 * <b>Note</b>: All IO implementations must extend the generic IO implementations.
 */
public abstract class GenericRollerSubsystem<Setpoint extends GenericRollerVoltage> extends SubsystemBase {

	/**
	 * Returns the current voltage setpoint of the rollers.
	 *
	 * @return The current voltage setpoint of the rollers.
	 */
	public abstract Setpoint getVoltageSetpoint();

	private String m_id;

	private GenericRollerIO m_io;
	protected GenericRollerIOInputsAutoLogged m_inputs = new GenericRollerIOInputsAutoLogged();

	private Alert m_disconnectedAlert;

	/**
	 * Constructs a new RollerSubsystem.
	 *
	 * @param io       The IO interface for hardware interactions.
	 * @param rollerId The roller ID, for example: ShooterMain, IntakeGround, etc.
	 */
	public GenericRollerSubsystem(String rollerId, GenericRollerIO io) {
		m_id = rollerId;
		m_io = io;
		m_disconnectedAlert = new Alert(m_id + " motor has disconnected!", AlertType.kError);
	}

	public void periodic() {
		Tracer.start(m_id + "_GenericRollerSubsystem_Periodic");

		m_io.updateInputs(m_inputs);
		Logger.processInputs(m_id + "/Inputs", m_inputs);

		m_disconnectedAlert.set(!m_inputs.motorConnected);

		m_io.runVolts(getVoltageSetpoint().voltage());
		Logger.recordOutput(m_id + "/VoltageSetpoint", getVoltageSetpoint().voltage());

		Tracer.finish(m_id + "_GenericRollerSubsystem_Periodic");
	}

	/**
	 * Returns the roller inputs.
	 *
	 * @return The roller inputs.
	 */
	public GenericRollerIOInputsAutoLogged getInputs() { return m_inputs; }

	/**
	 * The generic setpoint interface for the rollers.
	 *
	 * <p>
	 * <b>Usage</b>: Create an enum that implements this interface. Add a double parameter to each.
	 * Implement the voltage method.
	 */
	public interface GenericRollerVoltage {
		/**
		 * Returns the voltage at the goal.
		 *
		 * @return The voltage at the goal.
		 */
		double voltage();
	}
}
