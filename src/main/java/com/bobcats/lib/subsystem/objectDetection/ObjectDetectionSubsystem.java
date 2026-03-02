package com.bobcats.lib.subsystem.objectDetection;

import com.bobcats.lib.subsystem.objectDetection.ObjectDetectionIO.GamePieceObservation;
import com.bobcats.lib.subsystem.vision.LibVisionSubsystem;
import com.bobcats.lib.utils.Tracer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.dyn4j.geometry.Vector2;
import org.littletonrobotics.junction.Logger;

/**
 * A preset object detection subsystem used for locating game pieces on the field. Supports
 * Limelight, Photon and Photon Simulation.
 */
public class ObjectDetectionSubsystem extends SubsystemBase {
	private final ObjectDetectionIO[] m_cameras;
	private final ObjectDetectionIOInputsAutoLogged[] m_inputs;
	private final Alert[] m_disconnectedAlerts;

	private ArrayList<Pose2d> m_allPoses = new ArrayList<>();
	private ArrayList<GamePieceObservation> m_allDetections = new ArrayList<>();

	private final String m_gamePieceName;

	private boolean m_filterCloseMeasurements = false;
	private double m_filterThreshold = 0;

	private double m_maxDist = Double.POSITIVE_INFINITY;
	private boolean m_doFieldCheck = true;

	private boolean m_do3dLogging;
	private double m_3dFallenHeight, m_3dStandingHeight;
	private Rotation3d m_3dFallenRot, m_3dStandingRot;

	/**
	 * Constructs a new ObjectDetectionSubsystem.
	 *
	 * <p>
	 * <b>Note</b>: It's recommended to not put game piece cameras near the same FOV due to
	 * possible duplicate measurements. See {@link #setCloseMeasurementFilter(boolean, double)}.
	 *
	 * @param gamePieceName The name of the game piece being tracked.
	 * @param cameras       The IO interfaces to provide for each individual camera.
	 */
	public ObjectDetectionSubsystem(String gamePieceName, ObjectDetectionIO... cameras) {
		m_cameras = cameras;
		m_gamePieceName = gamePieceName;
		m_allPoses = new ArrayList<>();

		// Initialize inputs
		m_inputs = new ObjectDetectionIOInputsAutoLogged[cameras.length];
		for (int i = 0; i < m_inputs.length; i++) { m_inputs[i] = new ObjectDetectionIOInputsAutoLogged(); }

		// Initialize disconnected alerts for each camera
		m_disconnectedAlerts = new Alert[cameras.length];
		for (int i = 0; i < m_inputs.length; i++) {
			m_disconnectedAlerts[i] = new Alert(
					"Detection camera with index '" + Integer.toString(i) + "' is disconnected.", AlertType.kError);
		}
	}

	@Override
	public void periodic() {
		Tracer.start(m_gamePieceName + "_ObjectDet_Periodic");
		// Process inputs for each camera
		for (int i = 0; i < m_cameras.length; i++) {
			m_cameras[i].updateInputs(m_inputs[i]);
			Logger.processInputs("ObjectDetection/CameraData/Camera_i" + Integer.toString(i), m_inputs[i]);
		}

		ArrayList<Pose2d> allAcceptedPoses = new ArrayList<>();
		ArrayList<GamePieceObservation> allAcceptedDetections = new ArrayList<>();
		ArrayList<Pose2d> allRejectedPoses = new ArrayList<>();
		ArrayList<GamePieceObservation> allRejectedDetections = new ArrayList<>();

		for (int i = 0; i < m_cameras.length; i++) {
			var input = m_inputs[i];
			m_disconnectedAlerts[i].set(!input.isCameraConnected && m_cameras[i].isEnabled());

			for (var obs : input.targetObservations) {
				boolean rejected =
						// Field boundaries
						(m_doFieldCheck && !LibVisionSubsystem.kFieldRegion
								.contains(new Vector2(obs.fieldPose().getX(), obs.fieldPose().getY())))
								// Close measurement filter
								|| (m_filterCloseMeasurements && !allAcceptedPoses.stream()
										.noneMatch(p -> obs.fieldPose()
												.getTranslation()
												.getDistance(p.getTranslation()) <= m_filterThreshold))
								|| obs.distance() > m_maxDist;

				// Validate pose before accepting
				if (rejected) { allRejectedDetections.add(obs); allRejectedPoses.add(obs.fieldPose()); continue; }

				allAcceptedPoses.add(obs.fieldPose());
				allAcceptedDetections.add(obs);
			}
		}

		m_allPoses = allAcceptedPoses;
		m_allDetections = allAcceptedDetections;

		// Do logging
		if (m_do3dLogging) {
			// We have to use a different key 3d
			Logger.recordOutput("ObjectDetection/Detections/" + m_gamePieceName + "/Accepted3d",
					allAcceptedDetections.stream()
							.map(t -> new Pose3d(t.fieldPose().getX(), t.fieldPose().getY(),
									t.isTipped() ? m_3dFallenHeight : m_3dStandingHeight,
									t.isTipped() ? m_3dFallenRot : m_3dStandingRot))
							.toArray(Pose3d[]::new));
			Logger.recordOutput("ObjectDetection/Detections/" + m_gamePieceName + "/Rejected3d",
					allRejectedDetections.stream()
							.map(t -> new Pose3d(t.fieldPose().getX(), t.fieldPose().getY(),
									t.isTipped() ? m_3dFallenHeight : m_3dStandingHeight,
									t.isTipped() ? m_3dFallenRot : m_3dStandingRot))
							.toArray(Pose3d[]::new));
		} else {
			Logger.recordOutput("ObjectDetection/Detections/" + m_gamePieceName + "/Accepted2d",
					allAcceptedPoses.toArray(new Pose2d[0]));
			Logger.recordOutput("ObjectDetection/Detections/" + m_gamePieceName + "/Rejected2d",
					allRejectedPoses.toArray(new Pose2d[0]));
		}

		Tracer.finish(m_gamePieceName + "_ObjectDet_Periodic");
	}

	/**
	 * Sets the maximum detectable distance by the camera to avoid ambiguous detections.
	 *
	 * @param distanceMeters The new distance to filter beyond.
	 */
	public void setMaxDetectableDistance(double distanceMeters) { m_maxDist = distanceMeters; }

	/**
	 * Sets whether to enable or disable the out-of-bounds pose checks for rejection.
	 *
	 * @param isEnabled True to enable, false otherwise.
	 */
	public void setFieldBoundCheckEnabled(boolean isEnabled) { m_doFieldCheck = isEnabled; }

	/**
	 * Returns all of the detected game piece positions. Note that all positions will have a
	 * rotation of {@link Rotation2d#kZero}, so actual rotation is not measured.
	 *
	 * @return All of the game piece positions.
	 */
	public List<Pose2d> getDetectedInstances() { return m_allPoses; }

	/**
	 * Sets whether to do 3D or 2D pose logging for visualization. If {@code do3D} is set to false,
	 * ignore the other parameters. 3D allows for the display of individual game pieces via
	 * AdvantageScope.
	 *
	 * @param do3D                 Whether to do 3D logging.
	 * @param heightFallenMeters   The height of the center of the game piece when fallen.
	 * @param heightStandingMeters The height of the center of the game piece when standing.
	 * @param fallenRotation       The rotation of the game piece when fallen.
	 * @param standingRotation     The rotation of the game piece when standing.
	 */
	public void set3dPoseLogging(boolean do3D, double heightFallenMeters, double heightStandingMeters,
			Rotation3d fallenRotation, Rotation3d standingRotation) {
		m_do3dLogging = do3D;
		m_3dFallenHeight = heightFallenMeters;
		m_3dStandingHeight = heightStandingMeters;
		m_3dFallenRot = fallenRotation;
		m_3dStandingRot = standingRotation;
	}

	/**
	 * Attempts to find the closest game piece pose given the current robot pose. The supplier may
	 * return null if no game piece is present.
	 *
	 * @param botPose The field-relative robot pose supplier.
	 * @return The supplier for the closest game piece.
	 */
	public Supplier<Pose2d> getClosestGamePiece(Supplier<Pose2d> botPose) {
		return () -> m_allPoses.stream().min((p1, p2) -> {
			Pose2d pose = botPose.get();
			if (pose == null) return 0;
			double dp1 = p1.getTranslation().getDistance(pose.getTranslation());
			double dp2 = p2.getTranslation().getDistance(pose.getTranslation());
			return Double.compare(dp1, dp2);
		}).orElse(null);
	}

	/**
	 * Attempts to find the closest game piece pose given the current robot pose. May return null
	 * if no game piece is present.
	 *
	 * @param botPose The field-relative robot pose.
	 * @return The closest game piece.
	 */
	public Pose2d getClosestGamePiece(Pose2d botPose) {
		return getClosestGamePiece(() -> botPose).get();
	}

	/**
	 * Attempts to find the closest game piece pose given the current robot pose. The supplier may
	 * return null if no game piece is present.
	 *
	 * @param botPose  The field-relative robot pose supplier.
	 * @param isTipped Whether the detected game piece must be tipped/fallen over. Set to false to
	 *                 only detect upright game pieces.
	 * @return The supplier for the closest game piece.
	 */
	public Supplier<Pose2d> getClosestGamePiece(Supplier<Pose2d> botPose, boolean isTipped) {
		return () -> m_allDetections.stream()
				.filter(t -> t.isTipped() == isTipped)
				.map(t -> t.fieldPose())
				.min((p1, p2) -> {
					Pose2d pose = botPose.get();
					if (pose == null) return 0;
					double dp1 = p1.getTranslation().getDistance(pose.getTranslation());
					double dp2 = p2.getTranslation().getDistance(pose.getTranslation());
					return Double.compare(dp1, dp2);
				})
				.orElse(null);
	}

	/**
	 * Attempts to find the closest game piece pose given the current robot pose. May return null
	 * if no game piece is present.
	 *
	 * @param botPose  The field-relative robot pose.
	 * @param isTipped Whether the detected game piece must be tipped/fallen over. Set to false to
	 *                 only detect upright game pieces.
	 * @return The closest game piece.
	 */
	public Pose2d getClosestGamePiece(Pose2d botPose, boolean isTipped) {
		return getClosestGamePiece(() -> botPose, isTipped).get();
	}

	/**
	 * Sets the filter parameters. The filter attempts to remove close measurements, however this
	 * is not recommended as it may also filter out correct poses. The filter is disabled by
	 * default.
	 *
	 * @param enabled                 Whether to enable the filter.
	 * @param distanceThresholdMeters The distance at which the filter triggers, in meters.
	 */
	public void setCloseMeasurementFilter(boolean enabled, double distanceThresholdMeters) {
		m_filterCloseMeasurements = enabled;
		m_filterThreshold = distanceThresholdMeters;
	}

	/**
	 * Returns the cameras as an IO interface array.
	 *
	 * @return The camera array.
	 */
	public ObjectDetectionIO[] getCamerasIO() { return m_cameras; }

	/**
	 * Returns the camera inputs.
	 *
	 * @return The input array.
	 */
	public ObjectDetectionIOInputsAutoLogged[] getCameraInputs() { return m_inputs; }
}
