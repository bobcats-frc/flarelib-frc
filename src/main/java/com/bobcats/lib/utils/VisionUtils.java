package com.bobcats.lib.utils;

import static com.bobcats.lib.utils.Utils.EPSILON;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.TargetCorner;

/** A utility class for doing vision computations. */
public class VisionUtils {
	/** No instantiation for utils. */
	private VisionUtils() {
		throw new UnsupportedOperationException("Cannot instantiate utility class VisionUtils");
	}

	/**
	 * Returns whether the observed game piece has tipped/fallen over. Used for height compensation
	 * with fallen game pieces. Be careful when using an angled camera as the aspect ratios aren't
	 * the exact dimensions, but the projected dimensions.
	 *
	 * @param corners       The detected corners.
	 * @param minArStanding The minimum AR detected when standing, as the ratio W/H.
	 * @param maxArStanding The maximum AR detected when standing, as the ratio W/H.
	 * @return True if tipped, false otherwise.
	 */
	public static boolean isLikelyTipped(List<TargetCorner> corners, double minArStanding, double maxArStanding) {
		if (corners == null || corners.isEmpty()) return false;

		double minX = Double.POSITIVE_INFINITY, maxX = Double.NEGATIVE_INFINITY;
		double minY = Double.POSITIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY;
		for (TargetCorner c : corners) {
			minX = Math.min(minX, c.x);
			maxX = Math.max(maxX, c.x);
			minY = Math.min(minY, c.y);
			maxY = Math.max(maxY, c.y);
		}
		double width = maxX - minX;
		double height = maxY - minY;
		if (height <= EPSILON || width <= EPSILON) return false;
		double observedAR = width / height;

		// if observed AR deviates from the range, then assume tipped
		return observedAR > maxArStanding || observedAR < minArStanding;
	}

	/**
	 * Returns the aspect ratio of the target, given the corners. Returns the AR as the ratio of
	 * width / height.
	 *
	 * @param corners The detected corners.
	 * @return The aspect ratio.
	 */
	public static double getAspectRatio(List<TargetCorner> corners) {
		if (corners == null || corners.isEmpty()) return 0.0;

		double minX = Double.POSITIVE_INFINITY, maxX = Double.NEGATIVE_INFINITY;
		double minY = Double.POSITIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY;
		for (TargetCorner c : corners) {
			minX = Math.min(minX, c.x);
			maxX = Math.max(maxX, c.x);
			minY = Math.min(minY, c.y);
			maxY = Math.max(maxY, c.y);
		}
		double width = maxX - minX;
		double height = maxY - minY;
		if (height <= EPSILON || width <= EPSILON) return 0.0;
		double observedAR = width / height;

		return observedAR;
	}

	/**
	 * Finds the field-relative translation of the observed target.
	 *
	 * <p>
	 * To account for inaccuracies at high tx values, we use factors. Defined as
	 * <code>distance *= 1 + f_tan * |tan(tx)| + f_cos * cos(tx)</code>. You may need to negate the
	 * factors depending on your ty-tx conventions, however this is standard for the Limelight and
	 * PV.
	 *
	 * @param camPitchDeg                 The pitch of the camera relative to the horizontal, in
	 *                                    degrees.
	 * @param tyDeg                       The angular Y offset from the center of the target, in
	 *                                    degrees. (signed)
	 * @param targetHeightMeters          The height of the center of the target, in meters.
	 * @param lensCenterHeightMeters      The height of the center of the lens relative to the
	 *                                    floor, in meters.
	 * @param txDeg                       The angular X offset from the center of the target, in
	 *                                    degrees. (signed)
	 * @param cameraYawDeg                The yaw of the camera, in degrees.
	 * @param robotToCamera               The robot-to-camera transform.
	 * @param robotFieldPose              The field-relative pose of the robot.
	 * @param distCorrectionFactorTangent The distance correction factor for tan(tx).
	 * @param distCorrectionFactorCosine  The distance correction factor for cos(tx).
	 * @return The field-to-target translation.
	 */
	public static Translation2d getFieldToTarget(double camPitchDeg, double tyDeg, double targetHeightMeters,
			double lensCenterHeightMeters, double txDeg, double cameraYawDeg, Translation2d robotToCamera,
			Pose2d robotFieldPose, double distCorrectionFactorTangent, double distCorrectionFactorCosine) {
		double angleToTargetDeg = camPitchDeg + tyDeg;
		double angleToTargetRad = Math.toRadians(angleToTargetDeg);

		if (Math.abs(Math.tan(angleToTargetRad)) < EPSILON) return null;

		// Horizontal distance from camera to the vertical plane of the target
		double dist = (targetHeightMeters - lensCenterHeightMeters) / Math.tan(angleToTargetRad);

		// Bearing in camera frame (convert PV tx deg -> CCW rads).
		// PV tx: +R (CW). Convert to CCW: bearing = -tx_rad
		double bearingCameraRad = -Math.toRadians(txDeg);
		// We need a fudge factor to correct for skew at high tx angles
		dist *= 1.0 + distCorrectionFactorTangent * Math.abs(Math.tan(bearingCameraRad))
				+ distCorrectionFactorCosine * Math.cos(bearingCameraRad);

		// If camera has a yaw relative to robot forward, rotate the bearing into robot frame:
		double camYawRad = Math.toRadians(cameraYawDeg);
		double bearingRobotRad = bearingCameraRad + camYawRad;

		// Target position relative to camera in robot horizontal plane (camera F = +X, L = +Y)
		double xCam = dist * Math.cos(bearingRobotRad); // Forward
		double yCam = dist * Math.sin(bearingRobotRad); // Left

		// Add camera offset to get target in robot frame
		double xRobot = robotToCamera.getX() + xCam;
		double yRobot = robotToCamera.getY() + yCam;

		return robotFieldPose.transformBy(new Transform2d(xRobot, yRobot, new Rotation2d(bearingCameraRad)))
				.getTranslation();
	}

	/**
	 * Finds the field-relative translation of the observed target.
	 *
	 * @param camPitchDeg            The pitch of the camera relative to the horizontal, in
	 *                               degrees.
	 * @param tyDeg                  The angular Y offset from the center of the target, in
	 *                               degrees. (signed)
	 * @param targetHeightMeters     The height of the center of the target, in meters.
	 * @param lensCenterHeightMeters The height of the center of the lens relative to the floor, in
	 *                               meters.
	 * @param txDeg                  The angular X offset from the center of the target, in
	 *                               degrees. (signed)
	 * @param cameraYawDeg           The yaw of the camera, in degrees.
	 * @param robotToCamera          The robot-to-camera transform.
	 * @param robotFieldPose         The field-relative pose of the robot.
	 * @return The field-to-target translation.
	 */
	public static Translation2d getFieldToTarget(double camPitchDeg, double tyDeg, double targetHeightMeters,
			double lensCenterHeightMeters, double txDeg, double cameraYawDeg, Translation2d robotToCamera,
			Pose2d robotFieldPose) {
		return getFieldToTarget(camPitchDeg, tyDeg, targetHeightMeters, lensCenterHeightMeters, txDeg, cameraYawDeg,
				robotToCamera, robotFieldPose, 0, 0);
	}

	// Taken directly from PhotonVision.
	/**
	 * Returns a simulated camera properties instance with the given parameters.
	 *
	 * @param resWidth  The width of the image in pixels.
	 * @param resHeight The height of the image in pixels.
	 * @param fovDiag   The FOV.
	 * @param distCoeff The 8x1 distance coefficients for distortion.
	 * @return The calibrated properties.
	 */
	public static SimCameraProperties getCalibrated(int resWidth, int resHeight, Rotation2d fovDiag,
			Matrix<N8, N1> distCoeff) {
		SimCameraProperties props = new SimCameraProperties();

		if (fovDiag.getDegrees() <= 0 || fovDiag.getDegrees() >= 180) {
			fovDiag = Rotation2d.fromDegrees(MathUtil.clamp(fovDiag.getDegrees(), EPSILON, 180 - EPSILON));
			DriverStation.reportError("Requested invalid FOV! Clamping between (0, 180) degrees...", true);
		}
		double resDiag = Math.hypot(resWidth, resHeight);
		double diagRatio = Math.tan(fovDiag.getRadians() / 2);
		var fovWidth = new Rotation2d(Math.atan(diagRatio * (resWidth / resDiag)) * 2);
		var fovHeight = new Rotation2d(Math.atan(diagRatio * (resHeight / resDiag)) * 2);

		// assume centered principal point (pixels)
		double cx = resWidth / 2.0 - 0.5;
		double cy = resHeight / 2.0 - 0.5;

		// use given fov to determine focal point (pixels)
		double fx = cx / Math.tan(fovWidth.getRadians() / 2.0);
		double fy = cy / Math.tan(fovHeight.getRadians() / 2.0);

		// create camera intrinsics matrix
		var camIntrinsics = MatBuilder.fill(Nat.N3(), Nat.N3(), fx, 0, cx, 0, fy, cy, 0, 0, 1);
		props.setCalibration(resWidth, resHeight, camIntrinsics, distCoeff);

		return props;
	}

	/**
	 * Returns the diagonal FOV of the camera.
	 *
	 * @param horizontalFovDeg The horizontal FOV in degrees.
	 * @param verticalFovDeg   The veritcal FOV in degrees.
	 * @return The diagonal FOV in degrees.
	 */
	public static double diagonalFovDeg(double horizontalFovDeg, double verticalFovDeg) {
		double tanh = Math.tan(Math.toRadians(horizontalFovDeg) / 2.0);
		double tanv = Math.tan(Math.toRadians(verticalFovDeg) / 2.0);
		double diagHalf = Math.atan(Math.sqrt(tanh * tanh + tanv * tanv));
		return Math.toDegrees(2.0 * diagHalf);
	}
}
