// Copyright 2025-2027 Bobcats Robotics
// GitHub https://github.com/bobcats-frc
// This project is included under an MIT license by the LICENSE file located at
// the root project folder.
package com.bobcats.lib.container;

/**
 * A container for an integer value, used for passing by reference into a lambda expression.
 * Used to avoid errors.
 */
public class IntegerContainer {
	public int value;

	/**
	 * Constructs a new IntegerContainer.
	 *
	 * @param value The value to store.
	 */
	public IntegerContainer(int value) {
		this.value = value;
	}
}
