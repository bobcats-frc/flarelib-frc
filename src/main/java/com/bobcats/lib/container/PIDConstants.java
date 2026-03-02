package com.bobcats.lib.container;

/** A generic holder class for PID and IZone constants. */
public class PIDConstants {
	public final double kP, kI, kD, iZone;

	/** Constructs a new PIDConstants. */
	public PIDConstants(double kP, double kI, double kD, double iZone) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.iZone = iZone;
	}

	/** Constructs a new PIDConstants. */
	public PIDConstants(double kP, double kI, double kD) {
		this(kP, kI, kD, Double.POSITIVE_INFINITY);
	}

	/** Constructs a new PIDConstants. */
	public PIDConstants(double kP) {
		this(kP, 0, 0);
	}
}
