// Copyright 2025-2027 Bobcats Robotics
// GitHub https://github.com/bobcats-frc
// This project is included under an MIT license by the LICENSE file located at
// the root project folder.
package com.bobcats.lib.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import java.util.List;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Logs a flashing/strobing light to the dashboard, usually used with status indicators to get
 * the drivers' attention.
 */
public class LoggedFlashingDashboardColor {
	/** A record instance representing a condition-color pair. */
	public record FlashColor(BooleanSupplier condition, Color color) {}

	private final String m_key;
	private final List<FlashColor> m_colors;
	private final List<FlashColor> m_offColors;

	private int m_cycleCount = 0;
	private boolean m_flashOn = false;
	private final double m_cyclePeriod;
	private boolean m_activeWhelDisabled = false;

	public static double LoopPeriod = 0.02;

	private Color m_latestColor = Color.kBlack;

	/**
	 * Constructs a new FlashingColor.
	 *
	 * <p>
	 * Note: The first colors in the list take priority.
	 *
	 * @param ntKey              The NT key to log the color to.
	 * @param flashPeriod        The period of the flash, in seconds.
	 * @param colors             The map of colors to whether they should be active when the flash
	 *                           is considered on.
	 * @param offColor           The map of colors to whether they should be active when the flash
	 *                           is considered off.
	 * @param activeWhenDisabled Whether the flash should still be active when the robot is
	 *                           disabled.
	 */
	public LoggedFlashingDashboardColor(String ntKey, double flashPeriod, List<FlashColor> colors,
			List<FlashColor> offColor, boolean activeWhenDisabled) {
		m_key = ntKey;
		m_offColors = offColor;
		m_colors = colors;
		m_cyclePeriod = (int) (flashPeriod / LoopPeriod);
		m_activeWhelDisabled = activeWhenDisabled;

		if (m_cyclePeriod <= 0 || ntKey == null)
			throw new RuntimeException("Invalid params to FlashingColor: T = " + m_cyclePeriod + ", K = " + ntKey);
	}

	/**
	 * Constructs a new FlashingColor.
	 *
	 * <p>
	 * Note: The first colors in the list take priority.
	 *
	 * @param ntKey              The NT key to log the color to.
	 * @param flashPeriod        The period of the flash, in seconds.
	 * @param colors             The map of colors to whether they should be active when the flash
	 *                           is considered on.
	 * @param offColor           The color when the flash is considered off.
	 * @param activeWhenDisabled Whether the flash should still be active when the robot is
	 *                           disabled.
	 */
	public LoggedFlashingDashboardColor(String ntKey, double flashPeriod, List<FlashColor> colors, Color offColor,
			boolean activeWhenDisabled) {
		m_key = ntKey;
		m_offColors = List.of(new FlashColor(() -> true, offColor));
		m_colors = colors;
		m_cyclePeriod = (int) (flashPeriod / LoopPeriod);
		m_activeWhelDisabled = activeWhenDisabled;

		if (m_cyclePeriod <= 0 || ntKey == null)
			throw new RuntimeException("Invalid params to FlashingColor: T = " + m_cyclePeriod + ", K = " + ntKey);
	}

	/** Updates the color flash. */
	public void update() {
		Color color = Color.kBlack;

		// Step cycle
		m_cycleCount++;
		if (m_cycleCount > m_cyclePeriod) { m_cycleCount = 0; m_flashOn = !m_flashOn; }

		// Choose appropriate map
		boolean isOn = m_flashOn && (m_activeWhelDisabled || !DriverStation.isDisabled());
		List<FlashColor> colorMap = isOn ? m_colors : m_offColors;

		// Find first condition match
		for (var pair : colorMap) { if (pair.condition.getAsBoolean()) { color = pair.color; break; } }

		// Log data
		m_latestColor = color;
		Logger.recordOutput(m_key, color);
	}

	/**
	 * Returns the latest updated flash color.
	 *
	 * @return The latest updated flash color.
	 */
	public Color getLatestColor() { return m_latestColor; }

	/**
	 * Returns whether the flash is on.
	 *
	 * @return Whether the flash is on.
	 */
	public boolean isFlashOn() { return m_flashOn; }

	/**
	 * Returns whether the flash should still be active when the robot is disabled.
	 *
	 * @return Whether the flash should still be active when the robot is disabled.
	 */
	public boolean isActiveWhenDisabled() { return m_activeWhelDisabled; }

	/**
	 * Sets whether the flash should still be active when the robot is disabled.
	 *
	 * @param val Whether the flash should still be active when the robot is disabled.
	 */
	public void setActiveWhenDisabled(boolean val) { m_activeWhelDisabled = val; }
}
