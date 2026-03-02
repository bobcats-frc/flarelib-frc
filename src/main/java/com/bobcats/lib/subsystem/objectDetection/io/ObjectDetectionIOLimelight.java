package com.bobcats.lib.subsystem.objectDetection.io;

import com.bobcats.lib.container.LoggedTunableNumber;
import com.bobcats.lib.subsystem.objectDetection.ObjectDetectionIO;
import com.bobcats.lib.subsystem.vision.LimelightHelpers;
import com.bobcats.lib.subsystem.vision.LimelightHelpers.LimelightTarget_Detector;
import com.bobcats.lib.subsystem.vision.LimelightHelpers.RawDetection;
import com.bobcats.lib.utils.VisionUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.targeting.TargetCorner;

/** IO implementation for real Limelight hardware. */
public class ObjectDetectionIOLimelight implements ObjectDetectionIO {
	// NT Subscribers
	private final DoubleSubscriber m_latencySubscriber;

	// The name of the Limelight
	private final String m_name;

	private Supplier<Pose2d> m_robotPoseSupplier;
	private Translation2d m_robotToCamera;
	private double m_yaw, m_pitch;
	private double m_height;

	private double m_targetHeightMetersStanding;
	private double m_targetHeightMetersFallen;

	private double m_minAr;
	private double m_maxAr;

	private double m_maxYaw;

	private static final double kLatencyTimeoutMs = 250;

	private boolean m_enabled;

	public LoggedTunableNumber YawFudgeFactorTangent, YawFudgeFactorCosine;

	/**
	 * Constructs a new ObjectDetectionIOLimelight.
	 *
	 * <p>
	 * <b>Note</b>: The angles and coordinates follow WPILib conventions, meaning angles are CCW+,
	 * CW- and coordinates are Left+, Forward+. The camera should be mounted without any roll.
	 *
	 * @param name                       The name of the Limelight.
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
	 *                                   gets too big when entering the frame, or to avoid
	 *                                   ambiguity at high yaws.
	 */
	public ObjectDetectionIOLimelight(String name, Translation2d robotToCamera, Supplier<Pose2d> robotPoseSupplier,
			double camHeightGroundMeters, double camYawDeg, double camPitchDeg, double targetHeightMetersStanding,
			double targetHeightMetersFallen, double arMinStanding, double arMaxStanding, double maxYawDegrees) {
		m_name = name;

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

		// NT Subscribers
		var table = NetworkTableInstance.getDefault().getTable(name);
		m_latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);

		YawFudgeFactorTangent = new LoggedTunableNumber("ObjectDetection/LL/" + name + "/YawFudgeFactorTangent", 0.4);
		YawFudgeFactorCosine = new LoggedTunableNumber("ObjectDetection/LL/" + name + "/YawFudgeFactorCosine", -0.07);
	}

	@Override
	public void updateInputs(ObjectDetectionIOInputs inputs) {
		// Update connection status based on whether an update has been seen in the
		// given time range
		inputs.isCameraConnected = ((RobotController.getFPGATime() - m_latencySubscriber.getLastChange())
				/ 1000) < kLatencyTimeoutMs;

		// Object detection disabled, skip
		if (!m_enabled) { inputs.targetObservations = new GamePieceObservation[0]; return; }

		var results = LimelightHelpers.getLatestResults(m_name);

		ArrayList<GamePieceObservation> detections = new ArrayList<>();

		RawDetection[] rawDetections = LimelightHelpers.getRawDetections(m_name);

		int iterLength = rawDetections.length;
		if (rawDetections.length != results.targets_Detector.length) {
			DriverStation.reportWarning(
					"WARNING: ObjectDetectionIOLimelight::updateInputs, raw detections length doesn't match the results obtained from the latest results. Limited data may be used.",
					false);

			// Take the min length to avoid out of bounds exceptions
			iterLength = Math.min(rawDetections.length, results.targets_Detector.length);
		}

		for (int i = 0; i < iterLength; i++) {
			LimelightTarget_Detector target = results.targets_Detector[i];

			// Out of range, skip
			if (Math.abs(target.tx) > m_maxYaw) continue;

			// Convert from LL to PV corners
			List<TargetCorner> corners = new ArrayList<>();
			RawDetection det = rawDetections[i];
			corners.add(new TargetCorner(det.corner0_X, det.corner0_Y));
			corners.add(new TargetCorner(det.corner1_X, det.corner1_Y));
			corners.add(new TargetCorner(det.corner2_X, det.corner2_Y));
			corners.add(new TargetCorner(det.corner3_X, det.corner3_Y));

			double ar = VisionUtils.getAspectRatio(corners);

			// Check whether the game piece has fallen
			boolean isTipped = ar > m_maxAr || ar < m_minAr;

			// Get target height based on tipped-upright position
			double targHeight = isTipped ? m_targetHeightMetersFallen : m_targetHeightMetersStanding;

			Pose2d robotFieldPose = m_robotPoseSupplier.get();

			// Compute detection
			Translation2d fieldToTarget = VisionUtils.getFieldToTarget(m_pitch, target.ty, targHeight, m_height,
					target.tx, m_yaw, m_robotToCamera, m_robotPoseSupplier.get(), YawFudgeFactorTangent.get(),
					YawFudgeFactorCosine.get());

			// Invalid measurement
			if (fieldToTarget == null) {
				DriverStation.reportWarning(
						"WARNING: ObjectDetectionIOLimelight::updateInputs, null target pose received, this is likely due to the camera being incapable of measuring target distance at the current placement",
						false);
				continue;
			}

			// Add detection
			detections.add(new GamePieceObservation(Rotation2d.fromDegrees(target.tx),
					Rotation2d.fromDegrees(target.ty), robotFieldPose.getTranslation().getDistance(fieldToTarget),
					new Pose2d(fieldToTarget, Rotation2d.kZero), target.classID, results.timestamp_LIMELIGHT_publish,
					isTipped, ar));
		}

		inputs.targetObservations = detections.toArray(new GamePieceObservation[detections.size()]);
	}

	@Override
	public void setEnabled(boolean enabled) { m_enabled = enabled; }

	@Override
	public boolean isEnabled() { return m_enabled; }

	@Override
	public void setPipeline(int pipelineIndex) {
		LimelightHelpers.setPipelineIndex(m_name, pipelineIndex);
	}
}
