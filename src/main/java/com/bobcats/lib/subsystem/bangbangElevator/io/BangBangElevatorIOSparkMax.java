package com.bobcats.lib.subsystem.bangbangElevator.io;

import static com.revrobotics.spark.SparkUtils.retryUntilOk;

import com.bobcats.lib.subsystem.bangbangElevator.BangBangElevatorIO;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * The IO hardware implementation for bang-bang elevator hardware interacions with the
 * SparkMAX.
 */
public class BangBangElevatorIOSparkMax implements BangBangElevatorIO {
	private static final int kMaxAttempts = 5;

	private SparkMax m_sparkMain;
	private SparkMax m_sparkFollower;

	/**
	 * Constructs a new BangBangElevatorIOSparkMax.
	 *
	 * @param mainId           The CAN ID of the main SparkMAX.
	 * @param followerId       The CAN ID of the follower SparkMAX. (-1 if not present)
	 * @param mainInverted     Whether the main motor is inverted or not.
	 * @param followerInverted Whether the follower motor is inverted or not.
	 * @param currentLimitAmps The supply current limit for each motor, in Amps.
	 */
	public BangBangElevatorIOSparkMax(int mainId, int followerId, boolean mainInverted, boolean followerInverted,
			int currentLimitAmps) {
		if (mainId == -1) throw new IllegalStateException("main motor must be present (CAN: -1)");

		m_sparkMain = new SparkMax(mainId, MotorType.kBrushless);
		if (followerId != -1) m_sparkFollower = new SparkMax(followerId, MotorType.kBrushless);

		SparkMaxConfig mainConfig = new SparkMaxConfig();
		mainConfig.inverted(mainInverted);
		mainConfig.smartCurrentLimit(currentLimitAmps);
		mainConfig.idleMode(IdleMode.kBrake);

		SparkMaxConfig followerConfig = new SparkMaxConfig();
		if (followerId != -1) {
			boolean opposeMaster = followerInverted != mainInverted;
			followerConfig.smartCurrentLimit(currentLimitAmps);
			followerConfig.idleMode(IdleMode.kBrake);
			followerConfig.follow(mainId, opposeMaster);
		}

		retryUntilOk(
				() -> m_sparkMain.configure(mainConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters),
				kMaxAttempts, "Apply config to bang-bang elevator main motor (CAN " + mainId + ")");

		if (followerId != -1) retryUntilOk(
				() -> m_sparkFollower.configure(followerConfig, ResetMode.kResetSafeParameters,
						PersistMode.kPersistParameters),
				kMaxAttempts, "Apply config to bang-bang elevator follower motor (CAN " + followerId + ")");
	}

	@Override
	public void updateInputs(BangBangElevatorIOInputs inputs) {
		inputs.motorConnected = m_sparkMain.getLastError() == REVLibError.kOk;
		inputs.appliedVoltage = m_sparkMain.getAppliedOutput() * m_sparkMain.getBusVoltage();
		inputs.positionRevs = m_sparkMain.getEncoder().getPosition();
		inputs.velocityRPM = m_sparkMain.getEncoder().getVelocity();
		inputs.supplyCurrentAmps = Math.abs(m_sparkMain.getAppliedOutput() * m_sparkMain.getOutputCurrent());
		inputs.statorCurrentAmps = Math.abs(m_sparkMain.getOutputCurrent());
		inputs.temperatureCelsius = m_sparkMain.getMotorTemperature();
	}

	@Override
	public void runVolts(double volts) {
		m_sparkMain.setVoltage(volts);
	}
}
