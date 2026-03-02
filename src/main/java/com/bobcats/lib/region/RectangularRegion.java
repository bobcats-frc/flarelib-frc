package com.bobcats.lib.region;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A rectangular region on the field used to divide field into sections.
 */
public class RectangularRegion {
	private final Translation2d bottomLeft;
	private final Translation2d topRight;

	/**
	 * Create a 2D rectangular region.
	 *
	 * @param bottomLeft The bottom left corner of the rectangle.
	 * @param topRight   The top right corner of the rectangle.
	 */
	public RectangularRegion(Translation2d bottomLeft, Translation2d topRight) {
		this.bottomLeft = bottomLeft;
		this.topRight = topRight;
	}

	/**
	 * Returns the bottom left translation.
	 *
	 * @return The position.
	 */
	public Translation2d getBottomLeftPoint() { return bottomLeft; }

	/**
	 * Returns the top right translation.
	 *
	 * @return The position.
	 */
	public Translation2d getTopRightPoint() { return topRight; }

	/**
	 * Returns the bottom left X position.
	 *
	 * @return The position.
	 */
	public double getMinX() { return bottomLeft.getX(); }

	/**
	 * Returns the top right X position.
	 *
	 * @return The position.
	 */
	public double getMaxX() { return topRight.getX(); }

	/**
	 * Returns the bottom left Y position.
	 *
	 * @return The position.
	 */
	public double getMinY() { return bottomLeft.getY(); }

	/**
	 * Returns the top right Y position.
	 *
	 * @return The position.
	 */
	public double getMaxY() { return topRight.getY(); }

	/**
	 * Returns whether the given pose is bound within the region.
	 *
	 * @param pose The pose to check.
	 * @return True if bound within the region.
	 */
	public boolean isPoseWithinArea(Pose2d pose) {
		return pose.getX() >= bottomLeft.getX() && pose.getX() <= topRight.getX() && pose.getY() >= bottomLeft.getY()
				&& pose.getY() <= topRight.getY();
	}
}
