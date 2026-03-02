package com.ctre.phoenix6.configs;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

/**
 * A generic utility class for TalonFX and TalonFXS.
 */
public class TalonUtils {

	private TalonUtils() {}

	/**
	 * Gets the current configuration of the given TalonFX.
	 *
	 * @param fx The TalonFX to get the configuration from.
	 * @return The current configuration of the TalonFX.
	 */
	public static TalonFXConfiguration getTalonFXConfiguration(TalonFX fx) {
		/*
		 * Background Code: TalonFXConfiguration config = new TalonFXConfiguration(); StringBuilder
		 * serializedString = new StringBuilder(); StatusCode err =
		 * fx.getConfigurator().getConfigsPrivate(serializedString, 2); if (err == StatusCode.OK) {
		 * config.deserialize(serializedString.toString()); }
		 * fx.getConfigurator().getConfigsPrivate(null, 0);
		 */

		TalonFXConfiguration config = new TalonFXConfiguration();
		StatusCode status = fx.getConfigurator().refresh(config);
		if (status != StatusCode.OK) {
			DriverStation.reportWarning("Failed to refresh TalonFX configuration: " + status, true);
		}

		return config;
	}

	/**
	 * Gets the current configuration of the given TalonFXS.
	 *
	 * @param fxs The TalonFXS to get the configuration from.
	 * @return The current configuration of the TalonFXS.
	 */
	public static TalonFXSConfiguration getTalonFXSConfiguration(TalonFXS fxs) {
		TalonFXSConfiguration config = new TalonFXSConfiguration();
		StatusCode status = fxs.getConfigurator().refresh(config);
		if (status != StatusCode.OK) {
			DriverStation.reportWarning("Failed to refresh TalonFXS configuration: " + status, true);
		}

		return config;
	}

	/**
	 * Tries to apply the given function until a valid Ok status code is reached, or until the
	 * maximum number of tries is exceeded. The logged values can be checked from the
	 * {@link StatusCode} enum.
	 *
	 * @param applierFunction The function interacts with hardware and returns the status code.
	 * @param numMaxTries     The maximmum number of tries before canceling.
	 * @param desc            The description of the hardware interaction. For example:
	 *                        <code>"Applying the configuration to elevator motor #1"</code>
	 * @return True if an Ok status code was reached within the given attempts, false otherwise.
	 */
	public static boolean retryUntilOk(Supplier<StatusCode> applierFunction, int numMaxTries, String desc) {
		for (int i = 0; i < numMaxTries; i++) {
			StatusCode status = applierFunction.get();
			if (!status.isOK()) {
				DriverStation.reportWarning(desc + ": An error occured while executing (code " + status.value
						+ "), retrying (attempt #" + (i + 1) + " / " + numMaxTries, false);
				continue;
			}

			DriverStation.reportWarning(desc, false);
			return true;
		}

		DriverStation.reportWarning(desc + ": Failed to apply within " + numMaxTries + " attempts", true);
		return false;
	}

	/**
	 * Converts a boolean representing inversion into a MotorAlignmentValue for followers.
	 *
	 * @param isInverted Whether the follower is inverted.
	 * @return The corresponding MotorAlignmentValue.
	 */
	public static MotorAlignmentValue followerInversionFromBool(boolean isInverted) {
		return isInverted ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned;
	}
}
