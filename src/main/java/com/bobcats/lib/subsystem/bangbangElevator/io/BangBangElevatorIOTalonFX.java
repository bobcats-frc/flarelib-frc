package com.bobcats.lib.subsystem.bangbangElevator.io;

import static com.ctre.phoenix6.configs.TalonUtils.followerInversionFromBool;
import static com.ctre.phoenix6.configs.TalonUtils.retryUntilOk;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Revolutions;

import com.bobcats.lib.subsystem.bangbangElevator.BangBangElevatorIO;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
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
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The IO hardware implementation for bang-bang elevator hardware interacions with the TalonFX.
 */
public class BangBangElevatorIOTalonFX implements BangBangElevatorIO {
	private static final int kMaxAttempts = 5;

	private Debouncer m_motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

	private TalonFX m_mainTalon, m_followerTalon;

	private StatusSignal<Voltage> m_volts;
	private StatusSignal<AngularVelocity> m_velocity;
	private StatusSignal<Current> m_supplyCurrent;
	private StatusSignal<Current> m_statorCurrent;
	private StatusSignal<Angle> m_position;
	private StatusSignal<Temperature> m_temp;

	private VoltageOut m_voltageControl = new VoltageOut(0);
	private TorqueCurrentFOC m_currentControl = new TorqueCurrentFOC(0);

	private boolean m_isFOC;

	/**
	 * Constructs a new BangBangElevatorIOTalonFX.
	 *
	 * @param mainId                 The CAN ID of the main TalonFX.
	 * @param followerId             The CAN ID of the follower TalonFX. (-1 if not present)
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
	public BangBangElevatorIOTalonFX(int mainId, int followerId, boolean mainInverted, boolean followerInverted,
			double supplyCurrentLimitAmps, double statorCurrentLimitAmps, String mainCan, String followerCan,
			double peakFwdDutyCycle, double peakRevDutyCycle, boolean enableFOC) {
		m_mainTalon = new TalonFX(mainId, new CANBus(mainCan != null ? mainCan : ""));
		if (followerId != -1)
			m_followerTalon = new TalonFX(followerId, new CANBus(followerCan != null ? followerCan : ""));

		TalonFXConfiguration config = new TalonFXConfiguration();
		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimitAmps;
		config.CurrentLimits.StatorCurrentLimitEnable = true;
		config.CurrentLimits.StatorCurrentLimit = statorCurrentLimitAmps;
		config.MotorOutput.Inverted = mainInverted ? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;
		config.MotorOutput.PeakForwardDutyCycle = peakFwdDutyCycle;
		config.MotorOutput.PeakReverseDutyCycle = peakRevDutyCycle;
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		retryUntilOk(() -> m_mainTalon.getConfigurator().apply(config), kMaxAttempts,
				"Apply config to bang-bang elevator main motor (CAN " + mainId + ")");

		if (followerId != -1) {
			retryUntilOk(() -> m_followerTalon.getConfigurator().apply(config), kMaxAttempts,
					"Apply config to bang-bang elevator follower motor (CAN " + followerId + ")");
			retryUntilOk(
					() -> m_followerTalon.setControl(
							new Follower(mainId, followerInversionFromBool(mainInverted != followerInverted))),
					kMaxAttempts, "Setting bang-bang elevator follower to follower control (CAN " + followerId + ")");
		}

		m_isFOC = enableFOC;
		m_voltageControl = m_voltageControl.withEnableFOC(enableFOC);

		m_velocity = m_mainTalon.getVelocity();
		m_position = m_mainTalon.getPosition();
		m_volts = m_mainTalon.getMotorVoltage();
		m_supplyCurrent = m_mainTalon.getSupplyCurrent();
		m_statorCurrent = m_mainTalon.getStatorCurrent();
		m_temp = m_mainTalon.getDeviceTemp();

		BaseStatusSignal.setUpdateFrequencyForAll(100, m_velocity, m_position, m_volts, m_supplyCurrent,
				m_statorCurrent, m_temp);

		// NOTE: This disables every other signal except for those above
		if (m_followerTalon != null) ParentDevice.optimizeBusUtilizationForAll(m_mainTalon, m_followerTalon);
		else ParentDevice.optimizeBusUtilizationForAll(m_mainTalon);
	}

	@Override
	public void updateInputs(BangBangElevatorIOInputs inputs) {
		inputs.motorConnected = m_motorConnectedDebouncer.calculate(
				BaseStatusSignal.refreshAll(m_velocity, m_position, m_volts, m_supplyCurrent, m_statorCurrent, m_temp)
						.isOK());
		inputs.velocityRPM = m_velocity.getValue().in(RPM);
		inputs.positionRevs = m_position.getValue().in(Revolutions);
		inputs.appliedVoltage = m_volts.getValueAsDouble();
		inputs.supplyCurrentAmps = m_supplyCurrent.getValueAsDouble();
		inputs.statorCurrentAmps = m_statorCurrent.getValueAsDouble();
		inputs.temperatureCelsius = m_temp.getValue().in(Celsius);
	}

	@Override
	public void runVolts(double volts) {
		m_mainTalon.setControl(m_voltageControl.withOutput(volts));
	}

	@Override
	public void runTorqueCurrent(double amps) {
		if (!m_isFOC) {
			DriverStation.reportWarning(
					"WARNING: BangBangElevatorIOTalonFX::runTorqueCurrent, can't run TorqueCurrentFOC because FOC is not enabled",
					true);
			return;
		}

		m_mainTalon.setControl(m_currentControl.withOutput(amps));
	}
}
