package com.revrobotics.spark;

import com.revrobotics.REVLibError;
import com.revrobotics.jni.CANSparkJNI;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

/**
 * A static class to allow for for direct JNI SparkMax and SparkFlex interactions.
 */
public class SparkUtils {
	/** Constructor not avaiable. */
	private SparkUtils() {
		throw new UnsupportedOperationException("Cannot instantiate static class SparkUtils");
	}

	// Inversions //

	/**
	 * Sets whether the SparkMax is inverted or not.
	 *
	 * @param spark    The SparkMax.
	 * @param inverted True if inverted.
	 * @return The REVLibError code.
	 */
	public static REVLibError setInverted(SparkMax spark, boolean inverted) {
		return REVLibError.fromInt(CANSparkJNI.c_Spark_SetInverted(spark.sparkHandle, inverted));
	}

	/**
	 * Sets whether the SparkFlex is inverted or not.
	 *
	 * @param spark    The SparkFlex to use.
	 * @param inverted True if inverted.
	 * @return The REVLibError code.
	 */
	public static REVLibError setInverted(SparkFlex spark, boolean inverted) {
		return REVLibError.fromInt(CANSparkJNI.c_Spark_SetInverted(spark.sparkHandle, inverted));
	}

	/**
	 * Returns whether the SparkMax is inverted or not.
	 *
	 * @param spark The SparkMax.
	 * @return True if inverted.
	 */
	public static boolean getInverted(SparkMax spark) {
		return CANSparkJNI.c_Spark_GetInverted(spark.sparkHandle);
	}

	/**
	 * Returns whether the SparkFlex is inverted or not.
	 *
	 * @param spark The SparkFlex.
	 * @return True if inverted.
	 */
	public static boolean getInverted(SparkFlex spark) {
		return CANSparkJNI.c_Spark_GetInverted(spark.sparkHandle);
	}

	/**
	 * Tries to apply the given function until a valid Ok status code is reached, or until the
	 * maximum number of tries is exceeded. The logged values can be checked from the
	 * {@link REVLibError} enum.
	 *
	 * @param applierFunction The function interacts with hardware and returns the status code.
	 * @param numMaxTries     The maximmum number of tries before canceling.
	 * @param desc            The description of the hardware interaction. For example:
	 *                        <code>"Applying the configuration to elevator motor #1"</code>
	 * @return True if an Ok status code was reached within the given attempts, false otherwise.
	 */
	public static boolean retryUntilOk(Supplier<REVLibError> applierFunction, int numMaxTries, String desc) {
		for (int i = 0; i < numMaxTries; i++) {
			REVLibError status = applierFunction.get();
			if (status != REVLibError.kOk) {
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
}
