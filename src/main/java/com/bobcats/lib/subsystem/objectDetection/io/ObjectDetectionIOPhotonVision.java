package com.bobcats.lib.subsystem.objectDetection.io;

import com.bobcats.lib.container.LoggedTunableNumber;
import com.bobcats.lib.subsystem.objectDetection.ObjectDetectionIO;
import com.bobcats.lib.utils.VisionUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

/** IO implementation for real PhotonVision hardware. */
public class ObjectDetectionIOPhotonVision implements ObjectDetectionIO {
	public final PhotonCamera m_camera;
	private Supplier<Pose2d> m_robotPoseSupplier;
	private Translation2d m_robotToCamera;
	private double m_yaw, m_pitch;
	private double m_height;

	private double m_targetHeightMetersStanding;
	private double m_targetHeightMetersFallen;

	private double m_minAr;
	private double m_maxAr;

	private double m_maxYaw;

	/**
	 * A fallback option for when new data isn't received after a periodic update. Uses the
	 * previous data with a limit of {@link #kPreviousDataStaleUsageLimit} usages per stale update,
	 * usually for noise preventation. Defaults to true.
	 */
	public static boolean kUsePreviousDataIfNonePublished = true;
	/**
	 * For the fallback option {@link #kUsePreviousDataIfNonePublished}, the maximum usage amount
	 * of a stale reading before it's discarded. Defaults to a maximum of 4 usages per stale
	 * reading.
	 */
	public static int kPreviousDataStaleUsageLimit = 8;

	private List<PhotonPipelineResult> m_lastResults;
	private int m_lastDataUsageAmount;

	private boolean m_enabled;

	public LoggedTunableNumber YawFudgeFactorTangent, YawFudgeFactorCosine;

	/**
	 * Constructs a new ObjectDetectionIOPhotonVision.
	 *
	 * <p>
	 * <b>Note</b>: The angles and coordinates follow WPILib conventions, meaning angles are CCW+,
	 * CW- and coordinates are Left+, Forward+.
	 *
	 * @param name                       The name of the camera.
	 * @param robotToCamera              The 2D position of the camera relative to the robot (the
	 *                                   offset from the center).
	 * @param robotPoseSupplier          The field-relative robot pose supplier.
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
	 *                                   gets too big when entering the frame.
	 */
	public ObjectDetectionIOPhotonVision(String name, Translation2d robotToCamera, Supplier<Pose2d> robotPoseSupplier,
			double camHeightGroundMeters, double camYawDeg, double camPitchDeg, double targetHeightMetersStanding,
			double targetHeightMetersFallen, double arMinStanding, double arMaxStanding, double maxYawDegrees) {
		m_camera = new PhotonCamera(name);
		m_robotToCamera = robotToCamera;
		m_yaw = camYawDeg;
		m_pitch = -camPitchDeg;
		m_height = camHeightGroundMeters;

		m_targetHeightMetersStanding = targetHeightMetersStanding;
		m_targetHeightMetersFallen = targetHeightMetersFallen;

		m_minAr = arMinStanding;
		m_maxAr = arMaxStanding;

		m_maxYaw = maxYawDegrees;

		m_robotPoseSupplier = robotPoseSupplier;

		YawFudgeFactorTangent = new LoggedTunableNumber("ObjectDetection/PV/" + name + "/YawFudgeFactorTangent", 0.4);
		YawFudgeFactorCosine = new LoggedTunableNumber("ObjectDetection/PV/" + name + "/YawFudgeFactorCosine", -0.07);
	}

	@Override
	public void updateInputs(ObjectDetectionIOInputs inputs) {
		// Update connection status
		inputs.isCameraConnected = m_camera.isConnected();

		// Object detection disabled, skip
		if (!m_enabled) { inputs.targetObservations = new GamePieceObservation[0]; return; }

		// Read new camera observations
		var results = m_camera.getAllUnreadResults();

		if (!results.isEmpty()) {
			// Fresh data recorded
			m_lastDataUsageAmount = 0;
			m_lastResults = new ArrayList<PhotonPipelineResult>();
			m_lastResults.add(results.get(results.size() - 1));
		} else {
			// No fresh data, go to fallback
			if (kUsePreviousDataIfNonePublished && m_lastResults != null
					&& m_lastDataUsageAmount < kPreviousDataStaleUsageLimit) {
				// Re-use the previous data if none received
				results = m_lastResults;
				m_lastDataUsageAmount++;
			} else {
				// Nothing to do, give empty detections and return
				inputs.targetObservations = new GamePieceObservation[0];

				// Increment stale counter so we eventually stop trying
				m_lastDataUsageAmount++;
				return;
			}
		}

		// if (results.isEmpty()) { inputs.targetObservations = new GamePieceObservation[0]; return; }
		var latest = results.get(results.size() - 1);
		ArrayList<GamePieceObservation> detections = new ArrayList<>();

		for (var target : latest.targets) {
			// Out of range, skip
			if (Math.abs(target.yaw) > m_maxYaw) continue;

			double ar = VisionUtils.getAspectRatio(target.getDetectedCorners());

			// Check whether the game piece has fallen
			boolean isTipped = ar > m_maxAr || ar < m_minAr;

			// Get target height based on tipped-upright position
			double targHeight = isTipped ? m_targetHeightMetersFallen : m_targetHeightMetersStanding;

			Pose2d robotFieldPose = m_robotPoseSupplier.get();

			// Compute detection
			Translation2d fieldToTarget = VisionUtils.getFieldToTarget(m_pitch, target.getPitch(), targHeight, m_height,
					target.getYaw(), m_yaw, m_robotToCamera, robotFieldPose, YawFudgeFactorTangent.get(),
					YawFudgeFactorCosine.get());

			// Invalid measurement
			if (fieldToTarget == null) {
				DriverStation.reportWarning(
						"WARNING: ObjectDetectionIOPhotonVision::updateInputs, null target pose received, this is due to the camera being incapable of measuring target distance at the current placement position",
						false);
				continue;
			}

			// Register detection
			detections.add(new GamePieceObservation(Rotation2d.fromDegrees(target.getYaw()),
					Rotation2d.fromDegrees(target.getPitch()),
					robotFieldPose.getTranslation().getDistance(fieldToTarget),
					new Pose2d(fieldToTarget, Rotation2d.kZero), target.objDetectId, latest.getTimestampSeconds(),
					isTipped, ar));
		}

		inputs.targetObservations = detections.toArray(new GamePieceObservation[0]);
	}

	@Override
	public void setEnabled(boolean enabled) { m_enabled = enabled; }

	@Override
	public boolean isEnabled() { return m_enabled; }

	@Override
	public void setPipeline(int pipelineIndex) {
		m_camera.setPipelineIndex(pipelineIndex);
	}
}
