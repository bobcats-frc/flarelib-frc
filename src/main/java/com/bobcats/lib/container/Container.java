package com.bobcats.lib.container;

/**
 * Represents a class containing the given value.
 *
 * @param <T> The type of the variable to store.
 */
public class Container<T> {
	public T val;

	/**
	 * Constructs a new Container.
	 *
	 * @param def The default value.
	 */
	public Container(T def) {
		val = def;
	}
}
