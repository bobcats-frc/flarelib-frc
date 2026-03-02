package com.bobcats.lib.utils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.List;

/**
 * A utility class for vision calibration computations, to account for robot-to-camera
 * transform instead of pure manual measurements.
 *
 * <p>
 * <b>Note</b>: Data should be double-checked with manual measurements anyway to ensure
 * accuracy. Use as many samples as possible to iron out as much noise as possible.
 *
 * <p>
 * <b>Note</b>: This class assumes that the vision system provides accurate data, if your
 * vision measurements are too noisy, then the calibration will be noisy as well. Remember,
 * garbage in, garbage out.
 *
 * <p>
 * <b>Suggested Procedure</b>:
 * <ol>
 * <li>Pick a tag with a known field-relative pose that is parallel to the alliance walls.</li>
 * <li>Place the robot 1 meter away from the tag, make sure only 1 tag is visible.</li>
 * <li>Record the robot's precise field-relative pose.</li>
 * <li>Periodically compute as many samples as possible for the most accurate measurement.</li>
 * <li>Average the samples using {@link #averageCameraInRobotSamples(List)} to get the final
 * robot-to-camera transform.</li>
 * <li>Confirm accuracy by measuring the robot-to-camera transform manually and comparing.</li>
 * </ol>
 */
public final class VisionCalibration {

	/** No constructors for utility classes. */
	private VisionCalibration() {}

	/**
	 * Computes a single-sample robot-to-camera transform in the robot frame.
	 *
	 * @param fieldRobotPose The robot's known pose on the field (field-relative robot).
	 * @param fieldTagPose   The tag's known pose on the field (field-relative tag).
	 * @param tagPoseCam     The tag pose as reported by the vision pipeline in the camera frame
	 *                       (camera-relative tag).
	 * @return The Pose3d of the camera in the robot frame (camera-to-robot).
	 */
	public static Pose3d computeCameraInRobot(Pose3d fieldRobotPose, Pose3d fieldTagPose, Pose3d tagPoseCam) {

		// Simple transforms
		Transform3d fieldToTag = new Transform3d(new Pose3d(), fieldTagPose);
		Transform3d camToTag = new Transform3d(new Pose3d(), tagPoseCam);
		Transform3d fieldToCam = fieldToTag.plus(camToTag.inverse());
		Transform3d fieldToRobot = new Transform3d(new Pose3d(), fieldRobotPose);

		// R->C = (F->R)^-1 * (F->C)
		Transform3d robotToCam = fieldToRobot.inverse().plus(fieldToCam);

		// Convert to final Pose3d
		return new Pose3d(robotToCam.getTranslation(), robotToCam.getRotation());
	}

	/**
	 * Average multiple samples to reduce noise in measurement. This is recommended as vision
	 * measurements are usually too noisy for single-sample measurements.
	 *
	 * @param samples List of robot-to-camera Pose3d samples obtained from
	 *                {@link #computeCameraInRobot(Pose3d, Pose3d, Pose3d)}.
	 * @return Averaged final Pose3d sample.
	 */
	public static Pose3d averageCameraInRobotSamples(List<Pose3d> samples) {
		if (samples.isEmpty()) { throw new IllegalArgumentException("samples must not be empty"); }

		// Translation accumulator
		double sx = 0.0, sy = 0.0, sz = 0.0;

		// Quaternion accumulator
		double qw = 0.0, qx = 0.0, qy = 0.0, qz = 0.0;

		// Reference quaternion for hemisphere alignment
		Quaternion q0 = samples.get(0).getRotation().getQuaternion();
		double refW = q0.getW(), refX = q0.getX(), refY = q0.getY(), refZ = q0.getZ();

		for (Pose3d p : samples) {
			var t = p.getTranslation();
			sx += t.getX();
			sy += t.getY();
			sz += t.getZ();

			// See https://stackoverflow.com/questions/12374087/average-of-multiple-quaternions
			Quaternion q = p.getRotation().getQuaternion();
			// Hemisphere alignment, if dot(ref, q) < 0 then flip q
			double dot = refW * q.getW() + refX * q.getX() + refY * q.getY() + refZ * q.getZ();
			double w = q.getW(), x = q.getX(), y = q.getY(), z = q.getZ();
			if (dot < 0.0) { w = -w; x = -x; y = -y; z = -z; }
			qw += w;
			qx += x;
			qy += y;
			qz += z;
		}

		int n = samples.size();
		Translation3d avgTranslation = new Translation3d(sx / n, sy / n, sz / n);

		// Construct && normalize the quaternion
		double norm = Math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
		if (norm == 0.0) {
			// Fall back to first sample rotation if something went wrong
			return new Pose3d(avgTranslation, samples.get(0).getRotation());
		}
		qw /= norm;
		qx /= norm;
		qy /= norm;
		qz /= norm;
		Rotation3d avgRotation = new Rotation3d(new Quaternion(qw, qx, qy, qz));

		return new Pose3d(avgTranslation, avgRotation);
	}

	/**
	 * Converts the robot-to-camera Pose3d into a Transform3d for most common use cases.
	 *
	 * @param robotToCamera The robot-to-camera Pose3d.
	 * @return The robot-to-camera Transform3d.
	 */
	public static Transform3d robotToCameraAsTransform3d(Pose3d robotToCamera) {
		return new Transform3d(new Pose3d(), robotToCamera);
	}
}
