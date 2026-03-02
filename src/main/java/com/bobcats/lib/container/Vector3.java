package com.bobcats.lib.container;

import lombok.Getter;
import lombok.Setter;

/** A 3D vector class. */
@Getter
@Setter
public class Vector3 {
	public final double x;
	public final double y;
	public final double z;

	public static final Vector3 kZero = new Vector3(0, 0, 0);

	public Vector3(double x, double y, double z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public Vector3 copy() {
		return new Vector3(x, y, z);
	}

	public Vector3 add(Vector3 other) {
		return new Vector3(this.x + other.x, this.y + other.y, this.z + other.z);
	}

	public Vector3 sub(Vector3 other) {
		return new Vector3(this.x - other.x, this.y - other.y, this.z - other.z);
	}

	public Vector3 mul(Vector3 other) {
		return new Vector3(this.x * other.x, this.y * other.y, this.z * other.z);
	}

	public Vector3 scale(double scalar) {
		return new Vector3(this.x * scalar, this.y * scalar, this.z * scalar);
	}

	public double dot(Vector3 other) {
		return x * other.x + y * other.y + z * other.z;
	}

	public double norm() {
		return Math.sqrt(x * x + y * y + z * z);
	}

	@Override
	public String toString() {
		return "Vector3(" + "x=" + x + ", y=" + y + ", z=" + z + ')';
	}
}
