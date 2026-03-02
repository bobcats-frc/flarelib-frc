package com.bobcats.lib.subsystem.led;

import static edu.wpi.first.units.Units.Meters;

import com.bobcats.lib.utils.Tracer;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A preset LED subsystem.
 *
 * <p>
 * To apply patterns, use the {@link LibLEDSubsystem#applyPatternRunnable} method in order to
 * apply the patterns correctly.
 *
 * <p>
 * To apply a LEDPattern instance, use {@link LEDPatterns#generateRunnableFromPattern}. The
 * rest of the methods already return Runnables.
 *
 * <p>
 * <i> Note: In order to use simulation, select <b>Hardware -> Addressable LEDs</b> from the
 * topbar in the simulation UI.</i>
 */
public class LibLEDSubsystem extends SubsystemBase {
	private AddressableLED m_led;
	private AddressableLEDBuffer m_buffer;
	private double m_ledPixelDensity = 60;

	private Runnable m_defaultRunnable;

	private LEDPatterns m_patterns = new LEDPatterns(this);

	private int m_port;

	/**
	 * Constructs a new LibLEDSubsystem.
	 *
	 * @param ledPort      The port of the LED.
	 * @param lengthPixels The length of the LED in pixels.
	 * @param ledLength    The length of the LED.
	 */
	public LibLEDSubsystem(int ledPort, int lengthPixels, Distance ledLength) {
		m_led = new AddressableLED(ledPort);
		m_buffer = new AddressableLEDBuffer(lengthPixels);
		m_ledPixelDensity = lengthPixels / ledLength.in(Meters);
		m_led.setLength(m_buffer.getLength());
		m_led.start();
		resetLED();

		m_port = ledPort;
	}

	/**
	 * Constructs a new LibLEDSubsystem. Uses a default length of 60 pixels and a linear pixel
	 * density of 60 pixels per meter.
	 *
	 * @param ledPort The port of the LED.
	 */
	public LibLEDSubsystem(int ledPort) {
		m_led = new AddressableLED(ledPort);
		m_buffer = new AddressableLEDBuffer(60);
		m_led.setLength(m_buffer.getLength());
		m_led.start();
		resetLED();
	}

	@Override
	public void periodic() {
		Tracer.start("LEDs_Periodic_" + m_port);
		m_led.setData(m_buffer);

		if (m_defaultRunnable != null) { m_defaultRunnable.run(); }

		Tracer.finish("LEDs_Periodic_" + m_port);
	}

	/**
	 * Returns the pixel density of the LED, or how many pixels are in a meter.
	 *
	 * @return The density.
	 */
	public double getLEDPixelDensity() { return m_ledPixelDensity; }

	/**
	 * Returns the adressable LED.
	 *
	 * @return The LED.
	 */
	public AddressableLED getLED() { return m_led; }

	/**
	 * Returns the adressable LED's buffer.
	 *
	 * @return The buffer.
	 */
	public AddressableLEDBuffer getBuffer() { return m_buffer; }

	/**
	 * Returns the LED patterns class, which contains pattern presets and a method to apply them.
	 *
	 * @return The patterns class.
	 */
	public LEDPatterns getPatterns() { return m_patterns; }

	/**
	 * Applies the given buffer to the LED.
	 *
	 * @param customBuffer The custom buffer.
	 */
	public void setLEDBuffer(AddressableLEDBuffer customBuffer) {
		m_led.setData(customBuffer);
	}

	/**
	 * Resets the LED to a solid white color.
	 */
	public void resetLED() {
		applyPatternRunnable(m_patterns.generateRunnableFromPattern(m_patterns.solid(Color.kWhite)));
	}

	/**
	 * Applies a runnable to the LED.
	 *
	 * @param runnable The runnable function.
	 */
	public void applyPatternRunnable(Runnable runnable) {
		m_defaultRunnable = runnable;
	}
}
