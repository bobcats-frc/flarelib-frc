package com.bobcats.lib.subsystem.led;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.bobcats.lib.container.DoubleContainer;
import com.bobcats.lib.container.IntegerContainer;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.function.DoubleSupplier;

/** A container class holding LED patterns. */
public class LEDPatterns {
	private LibLEDSubsystem m_subsystem;

	/**
	 * Constructs a new LEDPatterns.
	 *
	 * @param subsystem The LED subsystem.
	 */
	public LEDPatterns(LibLEDSubsystem subsystem) {
		m_subsystem = subsystem;
	}

	/**
	 * Applies the given pattern to the LED.
	 *
	 * @param pattern The pattern to apply.
	 */
	public void apply(LEDPattern pattern) {
		if (pattern == null) return;
		pattern.applyTo(m_subsystem.getBuffer());
		m_subsystem.getLED().setData(m_subsystem.getBuffer());
	}

	//
	// ***Simple built-in LED patterns***
	//

	/**
	 * Applies a rainbow pattern to the LED subsystem.
	 *
	 * @param brightness The brightness on a scale of 0-255.
	 * @param saturation The saturation on a scale of 0-255.
	 * @param velocity   The velocity in m/s. (use 1 for default)
	 * @return The LEDPattern that is applied to the subsystem.
	 */
	public LEDPattern rainbow(double velocity, int brightness, int saturation) {
		LEDPattern pattern = LEDPattern.rainbow(saturation, brightness);
		Distance m_densityDist = Meters.of(1 / m_subsystem.getLEDPixelDensity());
		return pattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(velocity), m_densityDist);
	}

	/**
	 * Applies a solid color to the LED subsystem.
	 *
	 * @param color The color.
	 * @return The LEDPattern that is applied to the subsystem.
	 */
	public LEDPattern solid(Color color) {
		return LEDPattern.solid(color);
	}

	/**
	 * Creates a continuous gradient.
	 *
	 * @param colors The colors to include.
	 * @return The LEDPattern that is applied to the subsystem.
	 */
	public LEDPattern continuousGradient(Color... colors) {
		return LEDPattern.gradient(LEDPattern.GradientType.kContinuous, colors);
	}

	/**
	 * Creates a discontinuous gradient.
	 *
	 * @param colors The colors to include.
	 * @return The LEDPattern that is applied to the subsystem.
	 */
	public LEDPattern discontinuousGradient(Color... colors) {
		return LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, colors);
	}

	/**
	 * Creates a steps gradient where solid colors are displayed seperately at sections.
	 *
	 * @param steps The steps map, where the double is the point at which the section for the color
	 *              starts, has the range [0, 1).
	 * @return The LEDPattern that is applied to the subsystem.
	 */
	public LEDPattern steps(Map<Double, Color> steps) {
		return LEDPattern.steps(steps);
	}

	/**
	 * Masks a progress bar on the pattern provided, where only the supplied percentage will be
	 * shown.
	 *
	 * @param pattern          The pattern to apply the mask to.
	 * @param progressSupplier The percentage supplier.
	 * @return The LEDPattern that is applied to the subsystem with the mask.
	 */
	public LEDPattern progressMask(LEDPattern pattern, DoubleSupplier progressSupplier) {
		return pattern.mask(LEDPattern.progressMaskLayer(progressSupplier));
	}

	/**
	 * Generates a runnable that applies the given pattern.
	 *
	 * @param pattern The pattern to apply.
	 * @return The generated runnable.
	 */
	public Runnable generateRunnableFromPattern(LEDPattern pattern) {
		return () -> { apply(pattern); };
	}

	//
	// ***Methods below taken from HoundLib @ LEDPatterns.java***
	//

	/**
	 * Creates a pattern that triggers a bolt with a decreasing brightness tail across the LED.
	 * Cannot return a LEDPattern as the return value requires a consumer.
	 *
	 * <p>
	 * To use, call the function once to obtain the runnable. Then call the runnable periodically
	 * to update.
	 *
	 * @param color    The color of the bolt.
	 * @param length   The length of the bolt.
	 * @param time     The amount of time to take to finish the movement of the bolt across the LED
	 *                 strip, and optionally the time after which to send a new bolt.
	 * @param multiple Whether to continuously send bolts or not.
	 * @return The generated LED buffer runnable.
	 */
	public Runnable bolt(Color color, double length, double time, boolean multiple) {
		DoubleContainer previousTime = new DoubleContainer(0);
		// store fractional changes in the movement
		DoubleContainer movementBuffer = new DoubleContainer(0);
		IntegerContainer startPoint = new IntegerContainer(0);
		ExtendedColor extColor = new ExtendedColor(color);

		return () -> {
			double currentTime = Timer.getFPGATimestamp();
			double elapsedTime = currentTime - previousTime.value;
			previousTime.value = currentTime;
			// reset everything if it's been a while since we've run this continuously
			if (elapsedTime > 0.1) { elapsedTime = 0.0; startPoint.value = 0; movementBuffer.value = 0.0; }

			double stepPerLed = 255.0 / length;

			movementBuffer.value += elapsedTime * ((m_subsystem.getBuffer().getLength() + length) / time);
			if (movementBuffer.value > 1) {
				startPoint.value += (int) movementBuffer.value;
				if (multiple) startPoint.value %= (m_subsystem.getBuffer().getLength() + length);
				movementBuffer.value %= 1;
			}

			// start at the start point and work backwards
			// go till the end of the strip and cut off the end when i = 0
			for (int i = startPoint.value; i > startPoint.value - length && i >= 0; i--) {
				// if we're beyond the end of the strip, animate the final tail
				if (i < m_subsystem.getBuffer().getLength()) {
					int value = (int) (255 - (stepPerLed * (startPoint.value - i)));
					m_subsystem.getBuffer().setHSV(i, extColor.hue(), extColor.saturation(), value);
				}
			}
		};
	}

	/**
	 * Creates a pattern that moves a color between high and low brightness via a sine wave, which
	 * looks like "breathing".
	 *
	 * <p>
	 * To use, call the function once to obtain the runnable. Then call the runnable periodically
	 * to update.
	 *
	 * @param color         The color to breathe.
	 * @param onTime        The amount of time, in seconds, that the strip should be on for.
	 * @param minBrightness The brightness at the lowest point of the breath (make this 0 to turn
	 *                      off the strip).
	 * @param maxBrightness The brightness at the highest point of the breath.
	 * @return The generated LED buffer runnable.
	 */
	public Runnable breathe(Color color, double onTime, int minBrightness, int maxBrightness) {
		ExtendedColor extColor = new ExtendedColor(color);

		return () -> {
			double currentTime = Timer.getFPGATimestamp();
			// Calculate the sine wave phase based on the current time and onTime as the
			// period for one complete cycle
			double phase = (2 * Math.PI / onTime) * (currentTime % onTime);
			// Normalize sine wave output to oscillate between minBrightness and
			// maxBrightness
			int value = (int) (minBrightness + ((Math.sin(phase) + 1) / 2 * (maxBrightness - minBrightness)));

			for (int i = 0; i < m_subsystem.getBuffer().getLength(); i++) {
				m_subsystem.getBuffer().setHSV(i, extColor.hue(), extColor.saturation(), value);
			}
		};
	}

	/**
	 * Creates a pattern that creates a flickering effect similar to a fire, using a specfic
	 * palette of colors. Algorithm derivative of Fire2012.
	 *
	 * <p>
	 * To use, call the function once to obtain the runnable. Then call the runnable periodically
	 * to update.
	 *
	 * @param sparking        Percentage likelihood of a new spark being lit. higher chance = more
	 *                        roaring fire, lower chance = more flickery fire [0-1].
	 * @param cooling         How much the air cools as it rises. less cooling = taller flames,
	 *                        more cooling = shorter flames. [0-1].
	 * @param colors          The list of colors to interpolate between. the first color indicates
	 *                        the lowest temperature (typically black), and the last color
	 *                        indicates the highest temperature. A palette resembling default
	 *                        Fire2012 would be
	 *                        {@code List.of(Color.kBlack, Color.kRed, Color.kYellow, Color.kWhite)}.
	 * @param sectionInverted If true, the fire will be inverted.
	 * @return The generated LED buffer runnable.
	 */
	public Runnable fire(double sparking, double cooling, List<Color> colors, boolean sectionInverted) {
		Random random = new Random();
		int[] heat = new int[m_subsystem.getBuffer().getLength()];

		return () -> {
			// Cool down every cell a little
			for (int i = 0; i < m_subsystem.getBuffer().getLength(); i++) {
				heat[i] = Math.max(0,
						heat[i] - random.nextInt((int) (cooling * 10 * 255 / m_subsystem.getBuffer().getLength()) + 2));
			}

			// Heat from each cell drifts 'up' and diffuses a little
			for (int i = m_subsystem.getBuffer().getLength() - 1; i > 2; i--) {
				heat[i] = (heat[i - 1] + heat[i - 2] + heat[i - 2]) / 3;
			}

			// Randomly ignite new 'sparks' of heat near the bottom
			if (random.nextDouble() < sparking) {
				int y = random.nextInt(3);
				heat[y] = Math.min(heat[y] + random.nextInt(160, 255), 255);
			}

			// // Convert heat to LED colors

			for (int i = 0, j = 0; i < m_subsystem.getBuffer().getLength()
					&& j < m_subsystem.getBuffer().getLength(); i++, j++) {
				Color color = interpolateHeat(heat[i], colors);
				if (sectionInverted) m_subsystem.getBuffer().setLED(m_subsystem.getBuffer().getLength() - i, color);
				else m_subsystem.getBuffer().setLED(j, color);
			}
		};
	}

	/**
	 * Interpolates a fire color based on a palette of colors, from a heat value from 0-255.
	 *
	 * @param heat   The "heat" value as a temperature [0-255].
	 * @param colors The colors to use in the interpolation. the first color is the lowest heat,
	 *               and the last color is the highest heat. A good approximation to the original
	 *               Fire2012 animation is [Black, Red, Yellow, White].
	 * @return The color of the fire corresponding to that heat.
	 */
	private static Color interpolateHeat(int heat, List<Color> colors) {
		double scaled01 = heat / 255.0;

		double scaledValue = scaled01 * (colors.size() - 1);
		int index = (int) scaledValue;
		double t = scaledValue - index; // Fractional part of the scaled value

		// Perform linear interpolation
		Color color1 = colors.get(index);
		Color color2 = colors.get(Math.min(index + 1, colors.size() - 1));

		double red = interpolate(color1.red, color2.red, t);
		double green = interpolate(color1.green, color2.green, t);
		double blue = interpolate(color1.blue, color2.blue, t);

		return new Color(red, green, blue);
	}

	private static double interpolate(double start, double end, double t) {
		return start + (end - start) * t;
	}
}
