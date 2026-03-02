package com.bobcats.lib.utils;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Utilities assosciated with alliance color. */
public class AllianceUtil {
	private AllianceUtil() {}

	/**
	 * Flips the given blue-relative pose according to the current alliance.
	 *
	 * @param blueRelativePose The blue-alliance relative pose.
	 * @return The appropriately flipped pose.
	 */
	public static Pose2d flipWithAlliance(Pose2d blueRelativePose) {
		return isRedAlliance() ? FlippingUtil.flipFieldPose(blueRelativePose) : blueRelativePose;
	}

	/**
	 * Flips the given blue-relative pose according to the current alliance.
	 *
	 * @param blueRelativePose The blue-alliance relative translation.
	 * @return The appropriately flipped pose.
	 */
	public static Translation3d flipWithAlliance(Translation3d blueRelativeTranslation) {
		Pose2d pose = new Pose2d(blueRelativeTranslation.getX(), blueRelativeTranslation.getY(), Rotation2d.kZero);
		Pose2d flipped = isRedAlliance() ? FlippingUtil.flipFieldPose(pose) : pose;
		return new Translation3d(flipped.getX(), flipped.getY(), blueRelativeTranslation.getZ());
	}

	/**
	 * Flips the given blue-relative pose according to the current alliance.
	 *
	 * @param blueRelativePose The blue-alliance relative translation.
	 * @return The appropriately flipped pose.
	 */
	public static Translation2d flipWithAlliance(Translation2d blueRelativeTranslation) {
		Pose2d pose = new Pose2d(blueRelativeTranslation, Rotation2d.kZero);
		Pose2d flipped = isRedAlliance() ? FlippingUtil.flipFieldPose(pose) : pose;
		return new Translation2d(flipped.getX(), flipped.getY());
	}

	/**
	 * Returns whether the current alliance is red.
	 *
	 * @return Whether the current alliance is red.
	 */
	public static boolean isRedAlliance() {
		var opt = DriverStation.getAlliance();

		return opt.isPresent() && opt.get() == Alliance.Red;
	}
}
