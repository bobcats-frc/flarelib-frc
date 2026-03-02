package com.bobcats.lib.math;

import com.bobcats.lib.utils.Utils;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.function.Function;

/** A class containing utility methods for data analysis. */
public class DataTracker {
	private InterpolatingDoubleTreeMap m_data = new InterpolatingDoubleTreeMap();
	private Function<Double, Double> m_func;

	private double m_h = 1e-4;
	private double m_minX = Double.NEGATIVE_INFINITY, m_maxX = Double.POSITIVE_INFINITY;

	/**
	 * Constructs a new DataTracker.
	 *
	 * <p>
	 * Uses an InterpolatingDoubleTreeMap to store data points.
	 */
	public DataTracker() {}

	/***
	 * Constructs a new DataTracker.
	 *
	 * @param func The function to use for data analysis.
	 * @throws IllegalArgumentException If the function is null.
	 */
	public DataTracker(Function<Double, Double> func) {
		if (func == null) throw new IllegalArgumentException("data function cannot be null");
		m_func = func;
	}

	/**
	 * Adds a data point to the tracker. Only use if the function is not set, and you're using an
	 * interpolating map.
	 *
	 * @param x The X value of the data point.
	 * @param y The Y value of the data point.
	 */
	public void addDataPoint(double x, double y) {
		if (m_func != null) return;
		m_data.put(x, y);
		if (m_minX == Double.NEGATIVE_INFINITY && m_maxX == Double.POSITIVE_INFINITY) {
			// Initialize min and max X values
			m_minX = x;
			m_maxX = x;
		} else {
			m_minX = Math.min(m_minX, x);
			m_maxX = Math.max(m_maxX, x);
		}
	}

	/**
	 * Returns the standard deviation of the whole dataset. Returns 0 for a function tracker.
	 *
	 * @param samples The number of samples to take.
	 * @return The standard deviation.
	 */
	public double standardDeviation(int samples) {
		return standardDeviation(samples, m_minX, m_maxX);
	}

	/**
	 * Returns the standard deviation of the whole dataset. Returns 0 for a function tracker.
	 *
	 * @param samples The number of samples to take.
	 * @param min     The min X value to start from.
	 * @param max     The max X value to go to.
	 * @return The standard deviation.
	 */
	public double standardDeviation(int samples, double min, double max) {
		if (m_func != null) return 0.0;
		double avg = mean();

		double sumNum = 0.0;
		for (int i = 0; i < samples; i++) {
			double interpSample = min + (max - min) * i / samples;
			sumNum += Math.pow(interpSample - avg, 2);
		}

		return Math.sqrt(sumNum / samples);
	}

	/**
	 * Returns the derivative of the function at point X.
	 *
	 * @param x The point to differentiate at.
	 * @return The computed derivative.
	 */
	public double differentiate(double x) {
		// Via the definition of the derivative:
		// f'(x) = lim(h->0, (f(x + h) - f(x)) / h)
		return (f(x + m_h) - f(x)) / m_h;
	}

	/**
	 * Returns the mean across the given range.
	 *
	 * @param lowBound  The lower X bound.
	 * @param highBound The higher X bound.
	 * @return The mean value across the range.
	 */
	public double mean(double lowBound, double highBound) {
		if (Utils.epsilonEquals(lowBound, highBound)) return 0.0;
		// x_bar = int(f(x), hi, lo) / (hi - lo)
		return integrate(lowBound, highBound) / (highBound - lowBound);
	}

	/**
	 * Returns the mean across the whole dataset. Returns 0 for a function tracker.
	 *
	 * @return The mean value across the whole dataset.
	 */
	public double mean() {
		if (m_func != null) return 0.0;
		return mean(m_minX, m_maxX);
	}

	/**
	 * Integrates the function within the given bounds, and returns an approximate result. Uses the
	 * trapezoid rule.
	 *
	 * <p>
	 * This method returns the signed area under the curve, take the absolute value if you want
	 * only a positive result.
	 *
	 * @param lowBound  The lower X bound.
	 * @param highBound The higher X bound.
	 * @return The area under the curve.
	 */
	public double integrate(double lowBound, double highBound) {
		// 0 if the values are close
		if (Utils.epsilonEquals(lowBound, highBound)) return 0.0;

		// Switch and negate the values if L > H.
		boolean negate = false;
		if (lowBound > highBound) { double temp = lowBound; lowBound = highBound; highBound = temp; negate = true; }
		double integral = 0;
		int totalSteps = (int) ((highBound - lowBound) / m_h);
		double rem = (highBound - lowBound) - (totalSteps * m_h);

		// Iterate over range
		for (int i = 0; i < totalSteps; i++) {
			double x0 = lowBound + i * m_h; // New X value
			double x1 = x0 + m_h; // Next step X

			// Add the area of the trapezoid
			integral += 0.5 * (f(x0) + f(x1)) * m_h;
		}

		// Aproximate remainder trapezoidally
		if (rem > 0) { double x_i = lowBound + totalSteps * m_h; integral += 0.5 * (f(x_i) + f(highBound)) * rem; }

		return integral * (negate ? -1 : 1);
	}

	/**
	 * Integrates the function within the whole dataset, and returns an approximate result. Uses
	 * the trapezoid rule. Returns 0 for a function tracker.
	 *
	 * <p>
	 * This method returns the signed area under the curve, take the absolute value if you want
	 * only a positive result.
	 *
	 * @return The area under the curve.
	 */
	public double integrate() {
		if (m_func != null) return 0.0;
		return integrate(m_minX, m_maxX);
	}

	/**
	 * Sets the iterative step size. Defaults to 10^-4.
	 *
	 * @param newH The new step size.
	 */
	public void setH(double newH) { m_h = newH; }

	/**
	 * Returns the current step size.
	 *
	 * @return The current step size.
	 */
	public double getH() { return m_h; }

	/**
	 * Clears the dataset. Is a no-op for function tracker.
	 */
	public void clearDataset() {
		if (m_func != null) return; // No-op for function tracker
		// Clear the interpolating map
		m_data = new InterpolatingDoubleTreeMap();
		m_minX = Double.NEGATIVE_INFINITY;
		m_maxX = Double.POSITIVE_INFINITY;
	}

	private double f(double x) {
		return m_func != null ? m_func.apply(x) : m_data.get(x);
	}
}
