package com.bobcats.lib.subsystem.led;

import edu.wpi.first.wpilibj.util.Color;

/**
 * A wrapper class that provides conversions between RGB values and HSV values.
 */
public class ExtendedColor {
	/** The base Color object. */
	public final Color m_color;

	/**
	 * Creates a new ExtendedColor.
	 *
	 * @param color The {@link Color} object to wrap.
	 */
	public ExtendedColor(Color color) {
		m_color = color;
	}

	/**
	 * Gets the hue of the color.
	 *
	 * @return The hue, from 0-180.
	 */
	public final int hue() {
		return RGBtoHSV(m_color.red, m_color.green, m_color.blue)[0];
	}

	/**
	 * Gets the saturation of the color.
	 *
	 * @return The saturation, from 0-255
	 */
	public final int saturation() {
		return RGBtoHSV(m_color.red, m_color.green, m_color.blue)[1];
	}

	/**
	 * Gets the value of the color.
	 *
	 * @return The value, from 0-255
	 */
	public final int value() {
		return RGBtoHSV(m_color.red, m_color.green, m_color.blue)[2];
	}

	/**
	 * Gets the red value of the color.
	 *
	 * @return The red value, from 0-255
	 */
	public final int red() {
		return (int) (m_color.red * 255);
	}

	/**
	 * Gets the green value of the color.
	 *
	 * @return The green value, from 0-255
	 */
	public final int green() {
		return (int) (m_color.green * 255);
	}

	/**
	 * Gets the blue value of the color.
	 *
	 * @return The blue value, from 0-255
	 */
	public final int blue() {
		return (int) (m_color.blue * 255);
	}

	/**
	 * Converts an RGB color to HSV.
	 *
	 * @param r The red value of the color [0-1].
	 * @param g The green value of the color [0-1].
	 * @param b The blue value of the color [0-1].
	 */
	private static int[] RGBtoHSV(double red, double green, double blue) {
		double cMax = Math.max(red, Math.max(green, blue));
		double cMin = Math.min(red, Math.min(green, blue));

		double delta = cMax - cMin;

		int hue;
		if (delta == 0) {
			hue = 0;
		} else if (cMax == red) {
			hue = (int) Math.round(60 * (((green - blue) / delta + 6) % 6));
		} else if (cMax == green) {
			hue = (int) Math.round(60 * (((blue - red) / delta) + 2));
		} else { // cMax == blue
			hue = (int) Math.round(60 * (((red - green) / delta) + 4));
		}

		double saturation = (cMax == 0) ? 0 : delta / cMax;

		// Convert final values to correct range
		return new int[] { hue / 2, (int) Math.round(saturation * 255), (int) Math.round(cMax * 255) };
	}
}
