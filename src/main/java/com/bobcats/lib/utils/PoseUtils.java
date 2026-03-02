package com.bobcats.lib.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/** A utility class for working with 2D and 3D pose objects. */
public class PoseUtils {
	private PoseUtils() {
		throw new UnsupportedOperationException("Cannot instantiate static class PoseUtils");
	}

	/**
	 * Rotates the current pose around a point in 3D space.
	 *
	 * @param point The point in 3D space to rotate around.
	 * @param rot   The rotation to rotate the pose by.
	 * @return The new rotated pose.
	 */
	public static Pose3d rotateAround(Pose3d currentPose, Translation3d point, Rotation3d rot) {
		return new Pose3d(currentPose.getTranslation().rotateAround(point, rot),
				currentPose.getRotation().rotateBy(rot));
	}

	/**
	 * Offsets a Pose2d sideways by the given amount in meters.
	 *
	 * @param pose         The original poe.
	 * @param offsetMeters The amount to offset sideways (positive is to the right, negative is to
	 *                     the left).
	 * @return The new Pose2d after the offset.
	 */
	public static Pose2d offsetSideways(Pose2d pose, double offsetMeters) {
		// Get the current rotation
		Rotation2d rotation = pose.getRotation();

		// Compute the sideways (perpendicular) translation vector
		Translation2d offset = new Translation2d(offsetMeters * rotation.getSin(), -offsetMeters * rotation.getCos());

		// Apply the offset to the original translation
		Translation2d newTranslation = pose.getTranslation().plus(offset);

		// Return the new Pose2d with the updated translation and same rotation
		return new Pose2d(newTranslation, rotation);
	}

	/**
	 * Offsets a Pose3d sideways by the given amount in meters.
	 *
	 * @param pose         The original pose.
	 * @param offsetMeters The amount to offset sideways (positive is to the right, negative is to
	 *                     the left).
	 * @return The new Pose3d after the offset.
	 */
	public static Pose3d offsetSideways(Pose3d pose, double offsetMeters) {
		Rotation3d rotation = pose.getRotation();
		// Get the robot's local Y-axis direction (sideways) rotated into the global
		// frame
		Translation3d yDirection = new Translation3d(0, -1, 0).rotateBy(rotation);
		// Apply the offset
		Translation3d offset = yDirection.times(offsetMeters);
		Translation3d newTranslation = pose.getTranslation().plus(offset);
		return new Pose3d(newTranslation, rotation);
	}

	/**
	 * Offsets a Pose2d along the direction of the original pose.
	 *
	 * @param originalPose The orginal pose.
	 * @param distance     The distance to offset in meters.
	 * @return The offset position.
	 */
	public static Pose2d offsetPerpendicular(Pose2d originalPose, double distance) {
		Rotation2d heading = originalPose.getRotation();
		double dx = distance * heading.getCos();
		double dy = distance * heading.getSin();
		Translation2d newTranslation = originalPose.getTranslation().plus(new Translation2d(dx, dy));
		return new Pose2d(newTranslation, heading);
	}

	/**
	 * Offsets a Pose3d along the direction of the original pose.
	 *
	 * @param originalPose The original pose.
	 * @param distance     The distance to offset in meters.
	 * @return The offset pose.
	 */
	public static Pose3d offsetPerpendicular(Pose3d originalPose, double distance) {
		Rotation3d rotation = originalPose.getRotation();
		// Get the X-axis direction (forward) rotated into the global frame
		Translation3d xDirection = new Translation3d(1, 0, 0).rotateBy(rotation);
		// Scale by the negated distance and apply the offset
		Translation3d offset = xDirection.times(distance);
		Translation3d newTranslation = originalPose.getTranslation().plus(offset);
		return new Pose3d(newTranslation, rotation);
	}
}
