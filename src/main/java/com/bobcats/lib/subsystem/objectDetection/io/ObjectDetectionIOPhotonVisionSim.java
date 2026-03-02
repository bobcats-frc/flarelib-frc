package com.bobcats.lib.subsystem.objectDetection.io;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

/** IO implementation for simulation using PhotonVision's built-in simulator. */
public class ObjectDetectionIOPhotonVisionSim extends ObjectDetectionIOPhotonVision {
	private static HashMap<String, VisionSystemSim> m_visionSims = new HashMap<>();

	private final Supplier<Pose2d> m_poseSupplier;
	private final PhotonCameraSim m_cameraSim;

	private final Supplier<List<VisionTargetSim>> m_targets;

	private final String m_gamePiece;

	/**
	 * Constructs a new ObjectDetectionIOPhotonVisionSim.
	 *
	 * <p>
	 * <b>Note</b>: The angles and coordinates follow WPILib conventions, meaning angles are Left+,
	 * Right- and coordinates are Left+, Forward+. The camera should be mounted without any roll.
	 *
	 * @param name                       The name of the camera.
	 * @param gamePieceName              The name of the game piece.
	 * @param robotToCamera              The robot to camera 3D offset (measured from the robot's
	 *                                   center).
	 * @param poseSupplier               Supplier for the robot pose to use in simulation.
	 * @param estimatedPose              Supplier for the estimated robot pose.
	 * @param camHeightGroundMeters      The camera lens height off the ground, in meters.
	 * @param camYawDeg                  The yaw of the camera relative to the front of the robot,
	 *                                   in degrees.
	 * @param camPitchDeg                The pitch of the camera, in degrees.
	 * @param targetHeightMetersStanding The height of the target while standing, in meters.
	 * @param targetHeightMetersFallen   The height of the target while tipped, in meters.
	 * @param arMinStanding              The minimum aspect ratio of the target as W/H while
	 *                                   standing, in meters.
	 * @param arMaxStanding              The maximum aspect ratio of the target as W/H while
	 *                                   standing, in meters.
	 * @param maxYawDegrees              The maximum yaw to detect, other data is thrown out if a
	 *                                   tx greather than this value is detected. Generally used to
	 *                                   filter out samples where the Aspect Ratio of the target
	 *                                   gets too big when entering the frame, or to avoid
	 *                                   ambiguity at high yaws.
	 * @param cameraProperties           The simulation properties of the camera hardware.
	 * @param targets                    The supplier for the game piece targets.
	 */
	public ObjectDetectionIOPhotonVisionSim(String name, String gamePieceName, Transform3d robotToCamera,
			Supplier<Pose2d> poseSupplier, Supplier<Pose2d> estimatedPose, double camHeightGroundMeters,
			double camYawDeg, double camPitchDeg, double targetHeightMetersStanding, double targetHeightMetersFallen,
			double arMinStanding, double arMaxStanding, double maxYawDegrees, SimCameraProperties cameraProperties,
			Supplier<List<VisionTargetSim>> targets) {
		super(name, new Translation2d(robotToCamera.getX(), robotToCamera.getY()), estimatedPose, camHeightGroundMeters,
				camYawDeg, camPitchDeg, targetHeightMetersStanding, targetHeightMetersFallen, arMinStanding,
				arMaxStanding, maxYawDegrees);
		m_poseSupplier = poseSupplier;
		m_targets = targets;

		m_gamePiece = gamePieceName;

		// Initialize simulation (if not already created)
		if (m_visionSims.get(gamePieceName) == null) {
			m_visionSims.put(gamePieceName, new VisionSystemSim("main_objDet_" + gamePieceName));
			setTargets();
		}

		// Adds the simulation camera
		m_cameraSim = new PhotonCameraSim(m_camera, cameraProperties);
		m_cameraSim.enableDrawWireframe(true);
		m_cameraSim.enableRawStream(false);
		m_cameraSim.enableProcessedStream(true);
		m_visionSims.get(gamePieceName).clearAprilTags();
		m_visionSims.get(gamePieceName).addCamera(m_cameraSim, robotToCamera);

		setTargets();
		m_visionSims.get(m_gamePiece).update(m_poseSupplier.get());
	}

	@Override
	public void updateInputs(ObjectDetectionIOInputs inputs) {
		setTargets();
		if (super.isEnabled()) m_visionSims.get(m_gamePiece).update(m_poseSupplier.get());
		super.updateInputs(inputs);
	}

	@Override
	public void setEnabled(boolean enabled) {
		super.setEnabled(enabled);
	}

	private void setTargets() {
		m_visionSims.get(m_gamePiece).clearVisionTargets();

		for (var target : m_targets.get())
			m_visionSims.get(m_gamePiece).addVisionTargets(m_gamePiece, target);
	}
}
