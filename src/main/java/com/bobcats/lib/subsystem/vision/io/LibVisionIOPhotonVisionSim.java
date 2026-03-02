package com.bobcats.lib.subsystem.vision.io;

import com.bobcats.lib.subsystem.vision.LibVisionSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for simulation using PhotonVision's built-in simulator. */
public class LibVisionIOPhotonVisionSim extends LibVisionIOPhotonVision {
	private static VisionSystemSim m_visionSim;

	private Supplier<Pose2d> m_poseSupplier;
	private Supplier<Pose3d> m_poseSupplier3d;
	private final PhotonCameraSim m_cameraSim;

	/**
	 * Constructs a new LibVisionIOPhotonVisionSim.
	 *
	 * @param name             The name of the camera.
	 * @param robotToCamera    The 3D position of the camera relative to the robot (the offset from
	 *                         the center).
	 * @param poseSupplier     Supplier for the robot pose to use in simulation.
	 * @param poseSupplier3d   (Optional/Nullable) Supplier for the robot's 3D pose to use in
	 *                         simulation.
	 * @param cameraProperties The camera's properties.
	 */
	public LibVisionIOPhotonVisionSim(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier,
			Supplier<Pose3d> poseSupplier3d, SimCameraProperties cameraProperties) {
		super(name, robotToCamera);
		m_poseSupplier = poseSupplier;
		m_poseSupplier3d = poseSupplier3d;

		// Initialize simulation (if not already created)
		if (m_visionSim == null) {
			m_visionSim = new VisionSystemSim("main");
			m_visionSim.addAprilTags(LibVisionSubsystem.kLayout);
		}

		// Adds the simulation camera
		m_cameraSim = new PhotonCameraSim(m_camera, cameraProperties, LibVisionSubsystem.kLayout);
		m_cameraSim.enableDrawWireframe(true);
		m_cameraSim.enableRawStream(false);
		m_cameraSim.enableProcessedStream(true);
		m_visionSim.addCamera(m_cameraSim, robotToCamera);
	}

	@Override
	public void updateInputs(LibVisionIOInputs inputs) {
		if (m_poseSupplier3d == null) m_visionSim.update(m_poseSupplier.get());
		else m_visionSim.update(m_poseSupplier3d.get());
		super.updateInputs(inputs);
	}
}
