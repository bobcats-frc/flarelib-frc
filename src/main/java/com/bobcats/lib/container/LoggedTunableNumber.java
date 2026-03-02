// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.bobcats.lib.container;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not
 * or value is not in dashboard.
 */
public class LoggedTunableNumber implements DoubleSupplier {
	private final String m_key;
	private boolean m_hasDefault = false;
	private double m_defaultValue;
	private LoggedNetworkNumber m_dashboardNumber;
	private Map<Integer, Double> m_lastHasChangedValues = new HashMap<>();

	public static boolean DoTuning = false;

	/**
	 * Constructs a new LoggedTunableNumber.
	 *
	 * @param dashboardKey The NT key to use.
	 */
	public LoggedTunableNumber(String dashboardKey) {
		m_key = "/AdvantageKit/RealOutputs/" + dashboardKey;
	}

	/**
	 * Constructs a new LoggedTunableNumber.
	 *
	 * @param dashboardKey The NT key to use.
	 * @param defaultValue The default value.
	 */
	public LoggedTunableNumber(String dashboardKey, double defaultValue) {
		this(dashboardKey);
		initDefault(defaultValue);
	}

	/**
	 * Set the default value of the number. The default value can only be set once.
	 *
	 * @param defaultValue The default value.
	 */
	public void initDefault(double defaultValue) {
		if (!m_hasDefault) {
			m_hasDefault = true;
			m_defaultValue = defaultValue;
			if (DoTuning) m_dashboardNumber = new LoggedNetworkNumber(m_key, defaultValue);
		}
	}

	/**
	 * Gets the current value, from NT if available and in tuning mode. Otherwise returns the
	 * default value, or always 0.0 if no default was set.
	 *
	 * @return The current value
	 */
	public double get() {
		if (!m_hasDefault) return 0.0;
		else return DoTuning ? m_dashboardNumber.get() : m_defaultValue;
	}

	/**
	 * Checks whether the number has changed since the last check.
	 *
	 * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
	 *           objects. Recommended approach is to pass the result of "hashCode()".
	 * @return True if the number has changed since the last time this method was called, false
	 *         otherwise.
	 */
	public boolean hasChanged(int id) {
		double currentValue = get();
		Double lastValue = m_lastHasChangedValues.get(id);
		if (lastValue == null || currentValue != lastValue) {
			m_lastHasChangedValues.put(id, currentValue);
			return true;
		}

		return false;
	}

	/**
	 * Runs an action if any of the tunable numbers have changed.
	 *
	 * @param id             Unique identifier for the caller to avoid conflicts when shared
	 *                       between multiple objects. Recommended approach is to pass the result
	 *                       of "hashCode()".
	 * @param action         Callback to run when any of the tunable numbers have changed.
	 * @param tunableNumbers All tunable numbers to check.
	 */
	public static void ifChanged(int id, Consumer<double[]> action, LoggedTunableNumber... tunableNumbers) {
		if (Arrays.stream(tunableNumbers).anyMatch(tunableNumber -> tunableNumber.hasChanged(id)))
			action.accept(Arrays.stream(tunableNumbers).mapToDouble(LoggedTunableNumber::get).toArray());
	}

	/**
	 * Runs an action if any of the tunable numbers have changed.
	 *
	 * @param id             Unique identifier for the caller to avoid conflicts when shared
	 *                       between multiple objects. Recommended approach is to pass the result
	 *                       of "hashCode()".
	 * @param action         Callback to run when any of the tunable numbers have changed.
	 * @param tunableNumbers All tunable numbers to check.
	 */
	public static void ifChanged(int id, Runnable action, LoggedTunableNumber... tunableNumbers) {
		ifChanged(id, values -> action.run(), tunableNumbers);
	}

	@Override
	public double getAsDouble() { return get(); }
}
