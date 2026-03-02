package com.bobcats.lib.subsystem.rollers.io;

import static com.ctre.phoenix6.configs.TalonUtils.followerInversionFromBool;
import static com.ctre.phoenix6.configs.TalonUtils.retryUntilOk;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Volts;

import com.bobcats.lib.subsystem.rollers.GenericRollerIO;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** The IO hardware implementation for generic roller hardware interacions with the TalonFX. */
public class GenericRollerIOTalonFX implements GenericRollerIO {
	private static final int kMaxAttempts = 5;

	private Debouncer m_motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

	private TalonFX m_talonMain;
	private TalonFX m_talonFollower;

	private StatusSignal<Voltage> m_volts;
	private StatusSignal<Angle> m_position;
	private StatusSignal<AngularVelocity> m_velocity;
	private StatusSignal<Current> m_supplyCurrent;
	private StatusSignal<Current> m_statorCurrent;
	private StatusSignal<Temperature> m_temperature;

	private VoltageOut m_voltageControl = new VoltageOut(0);

	/**
	 * Constructs a new GenericRollerIOTalonFX.
	 *
	 * @param mainId                 The CAN ID of the main TalonFX.
	 * @param followerId             The CAN ID of the follower TalonFX: (-1 if not present)
	 * @param mainInverted           Whether the main motor is inverted or not.
	 * @param followerInverted       Whether the follower motor is inverted or not.
	 * @param supplyCurrentLimitAmps The supply current limit for each motor, in Amps.
	 * @param statorCurrentLimitAmps The stator current limit for each motor, in Amps.
	 * @param mainCan                <b>(Nullable)</b> The CAN Bus of the main TalonFX. Use null
	 *                               for the default CAN Bus.
	 * @param followerCan            <b>(Nullable)</b> The CAN Bus of the follower TalonFX. Use
	 *                               null for the default CAN Bus.
	 * @param peakFwdDutyCycle       The maximum forward duty cycle, should be in the range (0, 1].
	 * @param peakRevDutyCycle       The maximum reverse duty cycle, should be in the range [-1,
	 *                               0).
	 * @param enableFOC              Whether FOC should be enabled.
	 */
	public GenericRollerIOTalonFX(int mainId, int followerId, boolean mainInverted, boolean followerInverted,
			double supplyCurrentLimitAmps, double statorCurrentLimitAmps, String mainCan, String followerCan,
			double peakFwdDutyCycle, double peakRevDutyCycle, boolean enableFOC) {
		if (mainId == -1) throw new IllegalStateException("main motor must be present (CAN: -1)");

		m_talonMain = new TalonFX(mainId, new CANBus(mainCan != null ? mainCan : ""));
		if (followerId != -1)
			m_talonFollower = new TalonFX(followerId, new CANBus(followerCan != null ? followerCan : ""));

		TalonFXConfiguration config = new TalonFXConfiguration();
		// In FRC, CCW is positive by default
		config.MotorOutput.Inverted = mainInverted ? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;
		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimitAmps;
		config.CurrentLimits.StatorCurrentLimitEnable = true;
		config.CurrentLimits.StatorCurrentLimit = statorCurrentLimitAmps;
		config.MotorOutput.PeakForwardDutyCycle = peakFwdDutyCycle;
		config.MotorOutput.PeakReverseDutyCycle = peakRevDutyCycle;
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		config.MotorOutput.DutyCycleNeutralDeadband = 0.03;

		retryUntilOk(() -> m_talonMain.getConfigurator().apply(config), kMaxAttempts,
				"Apply config to generic roller main roller motor (CAN " + mainId + ")");

		if (followerId != -1) {
			retryUntilOk(() -> m_talonFollower.getConfigurator().apply(config), kMaxAttempts,
					"Apply config to generic roller follower roller motor (CAN " + followerId + ")");
			retryUntilOk(
					() -> m_talonFollower.setControl(
							new Follower(mainId, followerInversionFromBool(mainInverted != followerInverted))),
					kMaxAttempts, "Setting generic roller follower to follower control (CAN " + followerId + ")");
		}

		m_voltageControl = m_voltageControl.withEnableFOC(enableFOC);

		m_volts = m_talonMain.getMotorVoltage();
		m_position = m_talonMain.getPosition();
		m_velocity = m_talonMain.getVelocity();
		m_supplyCurrent = m_talonMain.getSupplyCurrent();
		m_statorCurrent = m_talonMain.getStatorCurrent();
		m_temperature = m_talonMain.getDeviceTemp();
		BaseStatusSignal.setUpdateFrequencyForAll(100, m_volts, m_position, m_velocity, m_supplyCurrent,
				m_statorCurrent, m_temperature);

		// NOTE: This disables every other signal except for those above
		if (m_talonFollower != null) ParentDevice.optimizeBusUtilizationForAll(m_talonMain, m_talonFollower);
		else ParentDevice.optimizeBusUtilizationForAll(m_talonMain);
	}

	@Override
	public void updateInputs(GenericRollerIOInputs inputs) {
		inputs.motorConnected = m_motorConnectedDebouncer.calculate(BaseStatusSignal
				.refreshAll(m_volts, m_position, m_velocity, m_supplyCurrent, m_statorCurrent, m_temperature)
				.isOK());
		inputs.appliedVoltage = m_volts.getValue().in(Volts);
		inputs.positionRevs = m_position.getValue().in(Revolutions);
		inputs.rpm = m_velocity.getValue().in(RPM);
		inputs.supplyCurrentAmps = Math.abs(m_supplyCurrent.getValue().in(Amps) * (m_talonFollower != null ? 2 : 1));
		inputs.statorCurrentAmps = Math.abs(m_statorCurrent.getValue().in(Amps) * (m_talonFollower != null ? 2 : 1));
		inputs.temperatureCelsius = m_temperature.getValue().in(Celsius);
	}

	@Override
	public void runVolts(double volts) {
		m_talonMain.setControl(m_voltageControl.withOutput(volts));
	}
}
