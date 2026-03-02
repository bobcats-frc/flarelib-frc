package com.bobcats.lib.utils;

import edu.wpi.first.wpilibj.Timer;
import java.util.concurrent.ConcurrentHashMap;
import org.littletonrobotics.junction.Logger;

/**
 * A simple class to help with tracing time between code sections.
 */
public class Tracer {
	private static final ConcurrentHashMap<String, Double> startTimes = new ConcurrentHashMap<>();

	/**
	 * Marks the start of a tracer section.
	 *
	 * @param name The name to use for the key.
	 */
	public static void start(String name) {
		startTimes.put(name, Timer.getFPGATimestamp());
	}

	/**
	 * Records the time since the start of this section to NetworkTables in milliseconds with the
	 * key "TracerTimes/&lt; name &gt;_ms". This is a no-op if start was never called with this
	 * section.
	 *
	 * @param name The name to use for the key.
	 */
	public static void finish(String name) {
		Double start = startTimes.remove(name);
		if (start == null) return;

		double deltaMs = (Timer.getFPGATimestamp() - start) * 1E+3;
		Logger.recordOutput("TracerTimes/" + name + "_ms", deltaMs);
	}
}
