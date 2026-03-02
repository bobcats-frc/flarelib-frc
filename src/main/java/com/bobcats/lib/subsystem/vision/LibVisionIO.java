package com.bobcats.lib.subsystem.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * The main superclass interface for vision camera (hardware) interaction IO classes.
 */
public interface LibVisionIO {

	/** The inputs class for the Vision Subsystem. */
	@AutoLog
	public static class LibVisionIOInputs {
		public boolean isCameraConnected = false;
		public TargetObservation latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d(),
				new Pose3d(), 0);
		public PoseObservation[] poseObservations = new PoseObservation[0];
		public int[] tagIDs = new int[0];
		public int bestTagID = -1;
	}

	/** Represents the angle to a simple target, not used for pose estimation. */
	public static record TargetObservation(Rotation2d tx, Rotation2d ty, Pose3d camToTarget, int tagId) {}

	/** Represents a robot pose sample used for pose estimation. */
	public static record PoseObservation(double timestamp, Pose3d pose, double ambiguity, int tagCount,
			double averageTagDistance, PoseObservationType type) {}

	/**
	 * An enum specifying which processing method is used when computing the pose.
	 */
	public static enum PoseObservationType {
		/** Multi-tag with limelight. Requires a gyro. */
		MEGATAG_2,
		/** Multi-tag with PhotonVision when available, single tag otherwise. */
		PHOTONVISION
	}

	/**
	 * Updates the given vision inputs.
	 *
	 * @param inputs The inputs.
	 */
	public default void updateInputs(LibVisionIOInputs inputs) {}

	/**
	 * Sets the current pipeline index.
	 *
	 * @param pipelineIndex The index.
	 */
	public default void setPipeline(int pipelineIndex) {};
}
