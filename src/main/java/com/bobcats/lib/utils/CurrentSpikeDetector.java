package com.bobcats.lib.utils;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import java.util.function.Supplier;

/**
 * A utility class to detect current spikes, useful for stall and object intake detection.
 */
public class CurrentSpikeDetector {
	private final Debouncer m_debouncer;
	private final Supplier<Double> m_currentSupplier;
	private double m_currentThreshold;

	/**
	 * Constructs a new CurrentSpikeDetector.
	 *
	 * @param currentThresholdAmps The current threshold at which it's considered a spike, in Amps.
	 * @param detectionTimeSeconds The time after the current threshold to consider it a spike, in
	 *                             seconds.
	 * @param currentSupplierAmps  The supplier of the current, in Amps.
	 */
	public CurrentSpikeDetector(double currentThresholdAmps, double detectionTimeSeconds,
			Supplier<Double> currentSupplierAmps) {
		m_debouncer = new Debouncer(detectionTimeSeconds, DebounceType.kRising);
		m_currentSupplier = currentSupplierAmps;
		m_currentThreshold = currentThresholdAmps;
	}

	/**
	 * Returns whether the current has spiked. Note that this uses the absolute value of the
	 * current so current in both directions is considered.
	 *
	 * @return True if the current has spiked, false otherwise.
	 */
	public boolean isSpiking() {
		return m_debouncer.calculate(Math.abs(m_currentSupplier.get()) > m_currentThreshold);
	}
}
