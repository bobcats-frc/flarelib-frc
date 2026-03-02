package com.bobcats.lib.math;

/**
 * A generic 2D bilinear interpolator over an axis-aligned grid. Takes in a matrix input, and
 * outputs a double corresponding to the interpolated result.
 */
public class BilinearInterpolator2D {
	private final double[] m_X, m_Y;
	private final double[][] m_V;

	private double m_minX = Double.POSITIVE_INFINITY, m_maxX = Double.NEGATIVE_INFINITY;

	/**
	 * Constructs a new BilinearInterpolator2D.
	 *
	 * <p>
	 * (Row i -> X[i], Col j -> Y[j])
	 *
	 * @param X <b>Sorted</b> X-axis grid of length M
	 * @param Y <b>Sorted</b> Y-axis grid of length N
	 * @param V MxN matrix of values: V[i][j] = f(X[i], Y[j])
	 */
	public BilinearInterpolator2D(double[] X, double[] Y, double[][] V) {
		if (X.length < 2 || Y.length < 2) throw new IllegalArgumentException("need at least 2 grid points per axis");
		if (V.length != X.length || V[0].length != Y.length)
			throw new IllegalArgumentException("value matrix dimensions must match X,Y lengths");
		m_X = X.clone();
		m_Y = Y.clone();
		m_V = new double[X.length][Y.length];
		for (int i = 0; i < X.length; i++) {
			m_minX = Math.min(m_minX, X[i]);
			m_maxX = Math.max(m_maxX, X[i]);
			System.arraycopy(V[i], 0, m_V[i], 0, Y.length);
		}
	}

	/**
	 * Perform bilinear interpolation at (x, y).
	 * <p>
	 * - If x or y is outside grid boundaries, extrapolates using nearest segment
	 *
	 * @param x The X coordinate.
	 * @param y The Y coordinate.
	 * @return The interpolated value at (x, y).
	 */
	public double interpolate(double x, double y) {
		// Find the indices
		int i1 = findUpperIndex(m_X, x);
		int i0 = i1 > 0 ? i1 - 1 : 0; // Safeguard against index underflow
		int j1 = findUpperIndex(m_Y, y);
		int j0 = j1 > 0 ? j1 - 1 : 0; // Safeguard against index underflow

		// Corner values
		double V00 = m_V[i0][j0];
		double V10 = m_V[i1][j0];
		double V01 = m_V[i0][j1];
		double V11 = m_V[i1][j1];

		// Fractional distances
		double x0 = m_X[i0], x1 = m_X[i1];
		double y0 = m_Y[j0], y1 = m_Y[j1];
		double tx = (x1 == x0) ? 0.0 : (x - x0) / (x1 - x0);
		double ty = (y1 == y0) ? 0.0 : (y - y0) / (y1 - y0);

		// Perform bilinear interpolation
		double v0 = V00 + tx * (V10 - V00);
		double v1 = V01 + tx * (V11 - V01);
		return v0 + ty * (v1 - v0);
	}

	/**
	 * Returns the minimum X value in the grid.
	 *
	 * @return The minimum X value.
	 */
	public double getMinX() { return m_minX; }

	/**
	 * Returns the maximum X value in the grid.
	 *
	 * @return The maximum X value.
	 */
	public double getMaxX() { return m_maxX; }

	// Helper: find smallest idx such that A[idx] >= x
	private static int findUpperIndex(double[] A, double x) {
		int lo = 0, hi = A.length - 1;

		if (x <= A[0]) return 0; // Left extrapolation
		if (x >= A[hi]) return hi; // Right extrapolation

		// Simple binary search
		while (lo + 1 < hi) {
			int mid = lo + (hi - lo) / 2; // Calculate midpoint, avoid int overflow
			if (A[mid] >= x) hi = mid;
			else lo = mid;
		}
		return hi;
	}
}
