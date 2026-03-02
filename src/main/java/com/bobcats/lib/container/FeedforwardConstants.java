package com.bobcats.lib.container;

/** A generic holder class for kS, kV and kA constants. */
public class FeedforwardConstants {
	public double kS, kV, kA;

	/** Constructs a new FeedforwardConstants. */
	public FeedforwardConstants(double kS, double kV, double kA) {
		this.kS = kS;
		this.kV = kV;
		this.kA = kA;
	}

	/** Constructs a new FeedforwardConstants. */
	public FeedforwardConstants(double kV, double kA) {
		this(0, kV, kA);
	}

	/** Constructs a new FeedforwardConstants. */
	public FeedforwardConstants(double kA) {
		this(kA, 0, 0);
	}
}
