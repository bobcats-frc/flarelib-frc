package com.bobcats.lib.subsystem.bangbangElevator;

import com.bobcats.lib.utils.Tracer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * A preset elevator controlled by a bang-bang style controller. Useful for linear mechanisms
 * like climbers that only need a fully extended or retracted state.
 */
public abstract class BangBangElevator extends SubsystemBase {

	/**
	 * Returns the setpoint of the elevator.
	 *
	 * @return The setpoint of the elevator.
	 */
	public abstract BangBangElevatorSetpoint getSetpoint();

	private BangBangElevatorSetpoint m_lastSetpoint = BangBangElevatorSetpoint.kRetract;

	private BangBangElevatorState m_state = BangBangElevatorState.kRetracted;
	private final BangBangElevatorIO m_io;
	private BangBangElevatorIOInputsAutoLogged m_inputs = new BangBangElevatorIOInputsAutoLogged();

	private final String m_name;

	private final Alert m_tempAlert;
	private final Alert m_disconnectAlert;

	private double m_forwardOutput;
	private boolean m_isTorqueCurrent;
	private double m_stallVelocity;
	private double m_stallTime;
	private double m_hitOutput;
	private boolean m_applyHitOutputWhenDown;

	private boolean m_torqueCurrentHoldVoltageSet = false;
	private double m_torqueCurrentHoldVoltage;

	private boolean m_tempLimitEnabled;
	private double m_maxTemp;

	private final Timer m_stallTimer = new Timer();

	private boolean m_stopRequest = false;
	private boolean m_holdRequest = false;

	/**
	 * Constructs a new BangBangElevator.
	 *
	 * @param io                     The IO interface.
	 * @param forwardOutput          The motor forward output. Uses volts if torque-current is
	 *                               disabled, uses Amps otherwise.
	 * @param isTorqueCurrent        Whether to run torque-current as the output. Keep in mind this
	 *                               only works with FOC-enabled CTRE hardware. Torque-current
	 *                               represents the per-motor torque-current, not the total
	 *                               torque-current.
	 * @param stallVelocity          The stall velocity in RPM.
	 * @param stallTime              The time after which the elevator is considered to be stalling
	 *                               while the elevator's velocity is less than or equal to the
	 *                               stall velocity.
	 * @param hitOutput              The output to apply to the motors once the elevator fully
	 *                               extends/retracts. Has the same units as forwardOutput. This is
	 *                               generally the kG value, and the elevator should not move with
	 *                               this output applied.
	 * @param applyHitOutputWhenDown Whether to apply the hit output when the elevator is
	 *                               retracted, in most cases set this to false.
	 * @param name                   The name of the elevator.
	 */
	public BangBangElevator(BangBangElevatorIO io, double forwardOutput, boolean isTorqueCurrent, double stallVelocity,
			double stallTime, double hitOutput, boolean applyHitOutputWhenDown, String name) {
		m_io = io;
		m_name = name;
		m_tempAlert = new Alert(name + " motor is overheating, system function may be limited!", AlertType.kWarning);
		m_disconnectAlert = new Alert(name + " motor has disconnected!", AlertType.kError);

		m_forwardOutput = forwardOutput;
		m_isTorqueCurrent = isTorqueCurrent;
		m_stallVelocity = stallVelocity;
		m_stallTime = stallTime;
		m_hitOutput = hitOutput;
		m_applyHitOutputWhenDown = applyHitOutputWhenDown;

		m_tempLimitEnabled = false;
		m_maxTemp = Double.POSITIVE_INFINITY;

		m_stallTimer.reset();
	}

	/**
	 * Constructs a new BangBangElevator.
	 *
	 * @param io                     The IO interface.
	 * @param forwardVolts           The motor forward output in Volts.
	 * @param stallVelocity          The stall velocity in RPM.
	 * @param stallTime              The time after which the elevator is considered to be stalling
	 *                               while the elevator's velocity is less than or equal to the
	 *                               stall velocity.
	 * @param hitOutput              The output to apply to the motors once the elevator fully
	 *                               extends/retracts. Has the same units as forwardOutput. This is
	 *                               generally the kG value, and the elevator should not move with
	 *                               this output applied.
	 * @param applyHitOutputWhenDown Whether to apply the hit output when the elevator is
	 *                               retracted, in most cases set this to false.
	 * @param name                   The name of the elevator.
	 */
	public BangBangElevator(BangBangElevatorIO io, double forwardVolts, double stallVelocity, double stallTime,
			double hitOutput, boolean applyHitOutputWhenDown, String name) {
		this(io, forwardVolts, false, stallVelocity, stallTime, hitOutput, applyHitOutputWhenDown, name);
	}

	// Public Methods //

	public void periodic() {
		Tracer.start(m_name + "_BangBangElevator_Periodic");

		// Update inputs
		m_io.updateInputs(m_inputs);
		Logger.processInputs(m_name + "/Inputs", m_inputs);

		// Update alerts
		m_disconnectAlert.set(!m_inputs.motorConnected);
		m_tempAlert.set(m_tempLimitEnabled && m_inputs.temperatureCelsius > m_maxTemp);

		var setpoint = getSetpoint();

		// Update setpoint if it has changed
		switch (setpoint) {
			case kExtend:
				// A new request to extend was sent
				if (m_lastSetpoint == BangBangElevatorSetpoint.kRetract) extend();
				break;

			case kRetract:
				// A new request to retract was sent
				if (m_lastSetpoint == BangBangElevatorSetpoint.kExtend) retract();
				break;

			default:
				// Invalid case
				DriverStation.reportWarning(
						"WARNING: BangBangElevator::periodic, invalid setpoint '" + setpoint + "' provided", false);
				stop();
				break;
		}
		m_lastSetpoint = setpoint;

		// If motor is disconnected, stop and return
		if (!m_inputs.motorConnected) { stop(); return; }

		// Overheating, stop driving if temperature limit exceeded
		if (m_tempLimitEnabled && m_inputs.temperatureCelsius > m_maxTemp) {
			// We specifically use applyOutput to actually prevent current when using torque-current in
			// order to prevent further overheating, though this behaves same as runVolts when using
			// voltage control and won't prevent current due to sliding down
			applyOutput(0);
			return;
		}

		// Only restart the stall timer if we are not in a stop or hold
		if (!m_stopRequest && !m_holdRequest && Math.abs(m_inputs.velocityRPM) > m_stallVelocity) {
			m_stallTimer.reset();
			m_stallTimer.start();
		}

		// We only consider a stall when not stopped or holding
		boolean stalled = !m_stopRequest && !m_holdRequest && m_stallTimer.hasElapsed(m_stallTime);

		switch (m_state) {
			case kExtending:
				// If we've stalled while extending, we've reached the extended limit
				if (stalled) {
					m_state = BangBangElevatorState.kExtended;
					// Clear stop/hold flags now that we've reached an end state
					m_stopRequest = false;
					m_holdRequest = false;
					m_stallTimer.stop();
					applyHoldIfAppropriate();
				} else if (m_holdRequest)
					// Hold-in-place while extending, apply hit output, do not change state
					applyHoldIfAppropriate();
				else if (m_stopRequest)
					// Full stop, zero output, do not change state
					m_io.runVolts(0);
				else
					// Normal extending behavior
					applyOutput(m_forwardOutput);
				break;

			case kRetracting:
				// If we've stalled while retracting, we've reached the retracted limit
				if (stalled) {
					m_state = BangBangElevatorState.kRetracted;
					// Clear stop/hold flags now that we've reached an end state
					m_stopRequest = false;
					m_holdRequest = false;
					m_stallTimer.stop();
					applyHoldIfAppropriate();
				} else if (m_holdRequest)
					// Hold-in-place while retracting, apply hit output, do not change state
					applyHoldIfAppropriate();
				else if (m_stopRequest)
					// Full stop, zero output, do not change state
					m_io.runVolts(0);
				else
					// Normal retracting behavior
					applyOutput(-m_forwardOutput);
				break;

			case kExtended:
				// Hold at top with hitOutput
				// If user requested stop, honor that (0.0), otherwise apply hold as configured
				if (m_stopRequest) m_io.runVolts(0);
				else applyHoldIfAppropriate();
				break;

			case kRetracted:
				// If applyHitOutputWhenDown is true, hold using hitOutput, otherwise zero output
				if (m_stopRequest) m_io.runVolts(0.0);
				else if (m_applyHitOutputWhenDown) applyHoldIfAppropriate();
				else m_io.runVolts(0);
				break;

			default:
				// Invalid case, shouldn't ever happen, just in case
				DriverStation.reportWarning(
						"WARNING: BangBangElevator::periodic, elevator state is invalid: " + m_state, false);
				m_io.runVolts(0);
				break;
		}

		Tracer.finish(m_name + "_BangBangElevator_Periodic");
	}

	/**
	 * Extends the elevator to its highest position.
	 */
	protected void extend() {
		// Only transition if not already extended or extending (accounting for stops/holds)
		if (m_state == BangBangElevatorState.kExtended
				|| (m_state == BangBangElevatorState.kExtending && !m_stopRequest && !m_holdRequest))
			return;

		m_state = BangBangElevatorState.kExtending;
		// Clear any stop flags when movement is commanded
		m_stopRequest = false;
		m_holdRequest = false;
		// Restart stall timer when commanded to move
		m_stallTimer.reset();
		m_stallTimer.start();
	}

	/**
	 * Retracts the elevator to its lowest position.
	 */
	protected void retract() {
		// Only transition if not already retracted or retracting (accounting for stops/holds)
		if (m_state == BangBangElevatorState.kRetracted
				|| (m_state == BangBangElevatorState.kRetracting && !m_stopRequest && !m_holdRequest))
			return;

		m_state = BangBangElevatorState.kRetracting;
		// Clear any stop flags when movement is commanded
		m_stopRequest = false;
		m_holdRequest = false;
		// Restart stall timer when commanded to move
		m_stallTimer.reset();
		m_stallTimer.start();
	}

	/**
	 * Stops the elevator. Keep in mind the elevator may slide downwards.
	 */
	public void stop() {
		// Stop driving the motors
		m_stopRequest = true;
		m_holdRequest = false;
		m_io.runVolts(0);
		// Stop the stall timer to prevent it from counting while stopped
		m_stallTimer.stop();
	}

	/**
	 * Stops the elevator and tries to hold its current position.
	 */
	public void stopInPlace() {
		if (m_isTorqueCurrent && !m_torqueCurrentHoldVoltageSet) {
			DriverStation.reportWarning(
					"WARNING: BangBangElevator::applyHoldIfAppropriate, can't use torque-current hold without setting a hold voltage via BangBangElevator::setTorqueCurrentHoldVoltage, stopping elevator",
					true);
			stop();
			return;
		}
		// Request a hold-in-place
		m_holdRequest = true;
		m_stopRequest = false;

		// Apply hit output
		applyHoldIfAppropriate();
	}

	/**
	 * Disables the stopped or hold modes internally.
	 */
	public void forceCancelStop() {
		m_stopRequest = false;
		m_holdRequest = false;
	}

	/**
	 * Overrides the forward output provided in the constructor.
	 *
	 * <p>
	 * Note: This clears the torque-current hold voltage if one was set, so if isTorqueCurrent is
	 * true, you need to set it via {@link #setTorqueCurrentHoldVoltage(double)} again.
	 *
	 * @param forwardOutput   The forward output. If torque-current is enabled, uses Amps,
	 *                        otherwise uses Volts.
	 * @param isTorqueCurrent Whether to run torque-current as the output. Keep in mind this only
	 *                        works with FOC-enabled CTRE hardware. Torque-current represents the
	 *                        per-motor torque-current, not the total torque-current.
	 */
	public void setForwardOutput(double forwardOutput, boolean isTorqueCurrent) {
		m_forwardOutput = forwardOutput;
		m_isTorqueCurrent = isTorqueCurrent;

		m_torqueCurrentHoldVoltageSet = false;
		m_torqueCurrentHoldVoltage = 0.0;
	}

	/**
	 * Overrides the hit output (the constant output after stalling) provided in the constructor.
	 *
	 * @param hitOutput              The output to apply to the motors once the elevator
	 *                               extends/retracts. Has the same units as forwardOutput.
	 * @param applyHitOutputWhenDown Whether to apply the hit output when the elevator is
	 *                               retracted, in most cases set this to false.
	 */
	public void setHitOutput(double hitOutput, boolean applyHitOutputWhenDown) {
		m_hitOutput = hitOutput;
		m_applyHitOutputWhenDown = applyHitOutputWhenDown;
	}

	/**
	 * Sets the voltage at which to hold the elevator at when in a hold state. This is only
	 * applicable when using torque-current control, otherwise it's a no-op. Not setting this when
	 * using torque-current will result in the {@link #stopInPlace()} method being disabled.
	 *
	 * <p>
	 * Since just using torque-current only maintains acceleration when holding (i.e. it only stops
	 * deceleration and doesn't hold a constant speed), this allows you to use the same
	 * functionality as voltage control when holding.
	 *
	 * @param holdVolts The voltage to hold the elevator at. The carriage should not move at all
	 *                  when using this voltage, but it should be just enough to counteract
	 *                  gravity.
	 */
	public void setTorqueCurrentHoldVoltage(double holdVolts) {
		if (!m_isTorqueCurrent) return;
		m_torqueCurrentHoldVoltage = holdVolts;
		m_torqueCurrentHoldVoltageSet = true;
	}

	/**
	 * Sets the temperature limits of the motors before a warning is displayed.
	 *
	 * @param enabled      Whether the limit is enabled.
	 * @param limitCelsius The temperature limit, in degrees Celsius. Ignore if enabled is set to
	 *                     false.
	 */
	public void setTemperatureLimits(boolean enabled, double limitCelsius) {
		m_tempLimitEnabled = enabled;
		m_maxTemp = limitCelsius;
	}

	/**
	 * Returns the elevator's current state.
	 *
	 * @return The elevator's current state.
	 */
	public BangBangElevatorState getState() { return m_state; }

	/**
	 * Returns the most recent elevator inputs.
	 *
	 * @return The most recent elevator inputs.
	 */
	public BangBangElevatorIOInputsAutoLogged getInputs() { return m_inputs; }

	/** An enum representing a bang-bang elevator's state. */
	public enum BangBangElevatorState {
		/** The elevator has fully extended. */
		kExtended,
		/** The elevator is in the process of extending, but may be stopped or holding position. */
		kExtending,
		/** The elevator is fully retracted. */
		kRetracted,
		/** The elevator is in the process of retracting, but may be stopped or holding position. */
		kRetracting;
	}

	/** An enum representing a bang-bang elevator's setpoint. */
	public enum BangBangElevatorSetpoint {
		/** Requests the elevator to expand fully. */
		kExtend,
		/** Requests the elevator to retract fully. */
		kRetract;
	}

	// Private Helpers //

	private void applyOutput(double output) {
		if (m_isTorqueCurrent) m_io.runTorqueCurrent(output);
		else m_io.runVolts(output);
	}

	private void applyHoldIfAppropriate() {
		// If disconnected or overheating, don't drive
		if (!m_inputs.motorConnected) return;
		if (m_tempLimitEnabled && m_inputs.temperatureCelsius > m_maxTemp) return;
		if (m_isTorqueCurrent && !m_torqueCurrentHoldVoltageSet) {
			DriverStation.reportWarning(
					"WARNING: BangBangElevator::applyHoldIfAppropriate, can't use torque-current hold without setting a hold voltage via BangBangElevator::setTorqueCurrentHoldVoltage, stopping elevator",
					true);
			stop();
			return;
		}

		// Apply hold output if in an end state
		if (m_state == BangBangElevatorState.kExtended) applyHitOutput();
		// If retracted, only apply hold if configured to do so
		else if (m_state == BangBangElevatorState.kRetracted && m_applyHitOutputWhenDown) applyHitOutput();
		// If retracted and not configured to apply hold, zero volts
		else if (m_state == BangBangElevatorState.kRetracted && !m_applyHitOutputWhenDown) m_io.runVolts(0);
		// For transitional states
		else applyHitOutput();
	}

	private void applyHitOutput() {
		double out = hitOutputForCurrentState();
		// Amps is only used in torque-current hold, otherwise volts
		if (m_isTorqueCurrent && !m_torqueCurrentHoldVoltageSet) m_io.runTorqueCurrent(out);
		else m_io.runVolts(out);
	}

	private double hitOutputForCurrentState() {
		// Same sign as the forwardOutput to maintain convetions
		return Math.copySign(
				m_isTorqueCurrent && m_torqueCurrentHoldVoltageSet ? m_torqueCurrentHoldVoltage : m_hitOutput,
				m_forwardOutput);
	}
}
