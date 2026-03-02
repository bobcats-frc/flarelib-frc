package com.bobcats.lib.subsystem.vision.io;

import com.bobcats.lib.subsystem.vision.LibVisionIO;
import com.bobcats.lib.subsystem.vision.LibVisionSubsystem;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

/** IO implementation for real PhotonVision hardware. */
public class LibVisionIOPhotonVision implements LibVisionIO {
	protected final PhotonCamera m_camera;
	private final Transform3d m_robotToCamera;

	/**
	 * A fallback option for when new data isn't received after a periodic update. Uses the
	 * previous data with a limit of {@link #kPreviousDataStaleUsageLimit} usages per stale update,
	 * usually for noise preventation. Defaults to false due to possible issues with the autonomous
	 * mode.
	 */
	public static boolean kUsePreviousDataIfNonePublished = false;
	/**
	 * For the fallback option {@link #kUsePreviousDataIfNonePublished}, the maximum usage amount
	 * of a stale reading before it's discarded. Defaults to a maximum of 16 usages per stale
	 * reading.
	 */
	public static int kPreviousDataStaleUsageLimit = 16;

	private List<PhotonPipelineResult> m_lastResults;
	private int m_lastDataUsageAmount;

	/**
	 * Constructs a new LibVisionIOPhotonVision.
	 *
	 * @param name          The name of the camera.
	 * @param robotToCamera The 3D position of the camera relative to the robot (the offset from
	 *                      the center).
	 */
	public LibVisionIOPhotonVision(String name, Transform3d robotToCamera) {
		m_camera = new PhotonCamera(name);
		m_robotToCamera = robotToCamera;
	}

	@Override
	public void updateInputs(LibVisionIOInputs inputs) {
		// Update connection status
		inputs.isCameraConnected = m_camera.isConnected();

		// Read new camera observations
		Set<Short> tagIDs = new HashSet<>();
		List<PoseObservation> poseObservations = new LinkedList<>();

		var results = m_camera.getAllUnreadResults();
		if (!results.isEmpty()) {
			// Fresh data recorded
			m_lastDataUsageAmount = 0;
			m_lastResults = results;
		} else {
			// No fresh data, go to fallback
			if (kUsePreviousDataIfNonePublished && m_lastResults != null
					&& m_lastDataUsageAmount < kPreviousDataStaleUsageLimit) {
				// Re-use the previous data if none received
				results = m_lastResults;
				m_lastDataUsageAmount++;
			} else {
				// Increment stale counter so we eventually stop trying
				m_lastDataUsageAmount++;
			}
		}

		for (var result : results) {
			// Update latest target observation
			if (result.hasTargets()) {
				inputs.latestTargetObservation = new TargetObservation(
						Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
						Rotation2d.fromDegrees(result.getBestTarget().getPitch()),
						new Pose3d(result.getBestTarget().bestCameraToTarget.getTranslation(),
								result.getBestTarget().bestCameraToTarget.getRotation()),
						result.getBestTarget().getFiducialId());
			} else {
				inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d(), Pose3d.kZero,
						0);
			}

			// Add pose observation
			if (result.multitagResult.isPresent()) { // Multitag result (if possible)
				var multitagResult = result.multitagResult.get();

				// Calculate robot pose
				Transform3d fieldToCamera = multitagResult.estimatedPose.best;
				Transform3d fieldToRobot = fieldToCamera.plus(m_robotToCamera.inverse());
				Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

				inputs.bestTagID = result.getBestTarget().getFiducialId();

				// Calculate total tag distance for an average
				double totalTagDistance = 0.0;
				for (var target : result.targets) {
					totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
				}

				// Add tag IDs
				tagIDs.addAll(multitagResult.fiducialIDsUsed);

				// Add observation
				poseObservations.add(new PoseObservation(result.getTimestampSeconds(), // Timestamp
						robotPose, // 3D pose estimate
						multitagResult.estimatedPose.ambiguity, // Pose ambiguity
						multitagResult.fiducialIDsUsed.size(), // Tag count
						totalTagDistance / result.targets.size(), // Average tag distance
						PoseObservationType.PHOTONVISION)); // Observation type

			} else if (!result.targets.isEmpty()) { // Single tag result
				var target = result.targets.get(0);

				// Calculate robot pose
				var tagPose = LibVisionSubsystem.kLayout.getTagPose(target.fiducialId);
				if (tagPose.isPresent()) {
					// Compute the field-relative robot pose
					Transform3d fieldToTarget = new Transform3d(tagPose.get().getTranslation(),
							tagPose.get().getRotation());
					Transform3d cameraToTarget = target.bestCameraToTarget;
					Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
					Transform3d fieldToRobot = fieldToCamera.plus(m_robotToCamera.inverse());
					Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());
					// Add tag ID
					tagIDs.add((short) target.fiducialId);

					inputs.bestTagID = target.fiducialId;

					// Add observation
					poseObservations.add(new PoseObservation(result.getTimestampSeconds(), // Timestamp
							robotPose, // 3D pose estimate
							target.poseAmbiguity, // Pose ambiguity
							1, // Tag count
							cameraToTarget.getTranslation().getNorm(), // Average tag distance
							PoseObservationType.PHOTONVISION)); // Observation type
				}
			}
		}

		// Save pose observations to inputs object
		inputs.poseObservations = new PoseObservation[poseObservations.size()];
		for (int i = 0; i < poseObservations.size(); i++) { inputs.poseObservations[i] = poseObservations.get(i); }

		// Save tag IDs to inputs objects
		inputs.tagIDs = new int[tagIDs.size()];
		int i = 0;
		for (int id : tagIDs) { inputs.tagIDs[i++] = id; }
	}

	@Override
	public void setPipeline(int pipelineIndex) {
		m_camera.setPipelineIndex(pipelineIndex);
	}
}
