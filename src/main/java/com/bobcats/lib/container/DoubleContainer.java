package com.bobcats.lib.container;

/**
 * A container for a double value, used for passing by reference into a lambda expression. Used
 * to avoid errors.
 */
public class DoubleContainer {
	public double value;

	/**
	 * Constructs a new DoubleContainer.
	 *
	 * @param value The value to store.
	 */
	public DoubleContainer(double value) {
		this.value = value;
	}
}
