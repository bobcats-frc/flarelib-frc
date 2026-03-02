package com.bobcats.lib.sim;

import com.bobcats.lib.utils.Utils;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.lang.reflect.Field;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

/** A utility class to simulate hardware. */
public class HardwareSimUtils {
	// Constants for current limiting
	private static final int kVoltsMaxSearchIters = 64;
	private static final double kDutyCycleDeadband = 1E-3;

	/** No constructor for utils classes. */
	private HardwareSimUtils() {
		throw new UnsupportedOperationException("can't instantiate utility class HardwareSimUtils");
	}

	/**
	 * Desaturates the requested voltage to stay within the provided current limits.
	 *
	 * @param gearbox          The DC motor gearbox. Do <b>not</b> use
	 *                         {@link DCMotor#withReduction(double)}.
	 * @param omegaRadPerSec   The angular velocity of the motor shaft, in rad/s.
	 * @param batteryVolts     The current voltage of the battery.
	 * @param requestedVolts   The voltage requested for the gearbox, in Volts.
	 * @param appliedVolts     The actual voltage last applied to the gearbox, in Volts.
	 * @param totalStatorLimit The total stator current limit, in Amps.
	 * @param totalSupplyLimit The total supply current limit, in Amps.
	 * @return The desaturated voltage to apply to the gearbox, in Volts.
	 */
	public static double desaturateVoltage(DCMotor gearbox, double omegaRadPerSec, double batteryVolts,
			double requestedVolts, double appliedVolts, double totalStatorLimit, double totalSupplyLimit) {
		// Calculate duty cycle
		double duty = (batteryVolts > 1e-12) ? Math.abs(requestedVolts) / batteryVolts : 1.0;
		duty = MathUtil.clamp(duty, -1.0, 1.0); // Clamp to [-1, 1] range

		// Safe lower bound on duty to avoid huge division when commanded voltage ≈ 0
		if (Math.abs(duty) < kDutyCycleDeadband) {
			duty = 1.0; // Treat tiny duty as "no division" so supply limit isn't artificially inflated
		}

		// Decide allowed current magnitude (min of supply and stator limits)
		double allowedStator = Math.abs(totalStatorLimit);
		double allowedSupplyToStator = Double.isInfinite(totalSupplyLimit) ? Double.POSITIVE_INFINITY
				: Math.abs(totalSupplyLimit / duty);
		double allowedMag = Math.min(allowedStator, allowedSupplyToStator);

		// Preview current at requested voltage
		double previewReq = gearbox.getCurrent(omegaRadPerSec, requestedVolts);

		boolean needLimit;
		if (Double.isInfinite(allowedMag)) {
			needLimit = false;
		} else {
			if (Math.abs(previewReq) <= allowedMag) {
				// Clearly within allowed => no limit
				needLimit = false;
			} else if (Math.abs(previewReq) > allowedMag) {
				// Clearly violating => limit
				needLimit = true;
			} else {
				// Use previous applied to decide (no new state field needed)
				// If previous applied was reduced (|applied| < |requested|) then remain limited
				needLimit = Math.abs(appliedVolts) < Math.abs(requestedVolts);
			}
		}

		// Determine applied voltage (binary-search inversion)
		double applied = requestedVolts;
		if (needLimit) {
			double target = Math.copySign(allowedMag, previewReq); // Preserve sign

			// Only apply if not browning out
			if (batteryVolts <= RoboRioSim.getBrownoutVoltage()) {
				applied = 0.0;
			} else {
				boolean positive = target > 0.0;
				double low = positive ? 0.0 : -batteryVolts;
				double high = positive ? batteryVolts : 0.0;

				// Endpoints
				double lowI = gearbox.getCurrent(omegaRadPerSec, low);
				double highI = gearbox.getCurrent(omegaRadPerSec, high);

				// If endpoints don't bracket target, pick closer endpoint
				if ((positive && highI < target) || (!positive && lowI > target)) {
					applied = Math.abs(highI - target) < Math.abs(lowI - target) ? high : low;
				} else {
					double mid = 0.0;
					for (int it = 0; it < kVoltsMaxSearchIters; ++it) {
						mid = 0.5 * (low + high);
						double midI = gearbox.getCurrent(omegaRadPerSec, mid);
						double err = midI - target;
						if (Math.abs(err) <= Utils.EPSILON) break;

						if (midI < target) low = mid;
						else high = mid;

					}
					applied = mid;
				}
			}
		}

		// Clamp applied to battery range
		return MathUtil.clamp(applied, -batteryVolts, batteryVolts);
	}

	/**
	 * Mofifies the {@link SimulatedBattery}'s {@link LinearFilter} filter to use a moving average
	 * filter with the specified number of taps. This number needs to be tuned for the combination
	 * of all mechanisms to match real behavior while preventing oscillations and large spikes.
	 *
	 * @param taps The number of taps for the moving average filter, defaults to 30.
	 */
	public static void modifySimulatedBatteryFilter(int taps) {
		if (taps <= 0) {
			DriverStation.reportWarning(
					"WARNING: HardwareSimUtils::modifySimulatedBatteryFilter, invalid taps value: " + taps, true);
			return;
		}
		try {
			// Obtain field and make it accessible
			Field f = SimulatedBattery.class.getDeclaredField("currentFilter");
			f.setAccessible(true);

			// Replace via unsafe
			try {
				Field unsafeField = sun.misc.Unsafe.class.getDeclaredField("theUnsafe");
				unsafeField.setAccessible(true);
				sun.misc.Unsafe unsafe = (sun.misc.Unsafe) unsafeField.get(null);

				long offset = unsafe.staticFieldOffset(f);
				Object base = unsafe.staticFieldBase(f);
				unsafe.putObject(base, offset, LinearFilter.movingAverage(taps));
				System.out.println("Replaced currentFilter via Unsafe.");
				return;
			} catch (Exception ex) {
				// last resort: try direct set (might fail if field is final)
				try {
					f.set(null, LinearFilter.movingAverage(taps));
					System.out.println("Replaced currentFilter with movingAverage(n) via direct set.");
					return;
				} catch (Exception finalEx) {
					DriverStation.reportWarning(
							"WARNING: HardwareSimUtils::modifySimulatedBatteryFilter, failed to replace SimulatedBattery.currentFilter",
							true);
				}
			}
		} catch (ReflectiveOperationException e) {
			DriverStation.reportWarning(
					"WARNING: HardwareSimUtils::modifySimulatedBatteryFilter, failed to access SimulatedBattery.currentFilter",
					true);
		}
	}
}
