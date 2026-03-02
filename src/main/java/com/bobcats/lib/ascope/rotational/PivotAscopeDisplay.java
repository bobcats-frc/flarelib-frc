package com.bobcats.lib.ascope.rotational;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Simulates an arm / pivot and calculates joint positions, publishing them to Network Tables
 * (via Akit Logger) for visualization in AdvantageScope.
 *
 * <p>
 * Note: Use {@link #PivotAscopeDisplay(String, Supplier, double, Supplier)} with length = 0
 * for a pivot display.
 */
public class PivotAscopeDisplay {
	private final String m_name;

	// Number of joints
	private final int m_jointCount;

	// Segment lengths for each arm segment
	private final double[] m_segmentLengths;

	// Suppliers for joint angles, rads
	private final Supplier<Rotation3d[]> m_jointAngles;

	// If the arm extends (telescopic arm), a supplier for extension
	private final Supplier<Double[]> m_extensionSupplier;

	// Base pose of the arm relative to the robot
	private final Supplier<Pose3d> m_basePose;

	/**
	 * Constructs a new PivotAscopeDisplay.
	 *
	 * @param name                 The name of the system.
	 * @param basePose             The pose supplier for the root of the arm.
	 * @param segmentLengthsMeters The length of each joint. Must have the same length as the joint
	 *                             angles and the extension supplier, and the joint count.
	 * @param jointAnglesRads      The supplier for the joint angles, in radians.
	 * @param extensionSupplier    The extensions supplier in meters, used for telescoping /
	 *                             extending arms.
	 * @throws IllegalArgumentException If the array lengths don't match.
	 */
	public PivotAscopeDisplay(String name, Supplier<Pose3d> basePose, double[] segmentLengthsMeters,
			Supplier<Rotation3d[]> jointAnglesRads, Supplier<Double[]> extensionSupplier) {
		if (extensionSupplier.get().length != segmentLengthsMeters.length
				|| jointAnglesRads.get().length != segmentLengthsMeters.length) {
			throw new IllegalArgumentException("Invalid length parameters provided to PivotAscopeDisplay");
		}

		m_name = name;
		m_jointCount = segmentLengthsMeters.length;
		m_segmentLengths = segmentLengthsMeters;
		m_jointAngles = jointAnglesRads;
		m_extensionSupplier = extensionSupplier;
		m_basePose = basePose;
	}

	/**
	 * Constructs a single-jointed new PivotAscopeDisplay.
	 *
	 * @param name              The name of the system.
	 * @param basePose          The pose supplier for the root of the arm.
	 * @param length            The length of the arm in meters.
	 * @param angleSupplier     The supplier for the arm angle, in radians.
	 * @param extensionSupplier The extensions supplier in meters, used for telescoping / extending
	 *                          arms.
	 * @throws IllegalArgumentException If the array lengths don't match.
	 */
	public PivotAscopeDisplay(String name, Supplier<Pose3d> basePose, double length, Supplier<Rotation3d> angleSupplier,
			Supplier<Double> extensionSupplier) {
		this(name, basePose, new double[] { length }, () -> new Rotation3d[] { angleSupplier.get() },
				() -> new Double[] { extensionSupplier.get() });
	}

	/**
	 * Constructs a new single-jointed PivotAscopeDisplay.
	 *
	 * <p>
	 * Note: Use length = 0 for pivot.
	 *
	 * @param name          The name of the system.
	 * @param basePose      The pose supplier for the root of the arm.
	 * @param length        The length of the arm in meters.
	 * @param angleSupplier The supplier for the arm angle, in radians.
	 */
	public PivotAscopeDisplay(String name, Supplier<Pose3d> basePose, double length,
			Supplier<Rotation3d> angleSupplier) {
		this(name, basePose, new double[] { length }, () -> new Rotation3d[] { angleSupplier.get() },
				() -> new Double[] { 0.0 });
	}

	/**
	 * Computes the final pose of the arm's end effector using forward kinematics.
	 *
	 * @return The pose of the end effector.
	 */
	public Pose3d computeEndEffectorPose() {
		return getJointPose(m_jointCount, new Transform3d());
	}

	/**
	 * Computes the final pose of the arm's end effector using forward kinematics.
	 *
	 * @param offset The offset applied to the final position.
	 * @return The pose of the end effector.
	 */
	public Pose3d computeEndEffectorPose(Transform3d offset) {
		return getJointPose(m_jointCount, offset);
	}

	/**
	 * Returns the pose of the specified joint (1-indexed). Joint 1 is the first joint (i.e. the
	 * transformation after the base, including the first segment's rotation and translation).
	 *
	 * @param joint  The joint number. (1-indexed)
	 * @param offset The offset applied to the final position.
	 * @return The pose of that joint, computed using forward kinematics.
	 * @throws IllegalArgumentException if the joint number is invalid.
	 */
	public Pose3d getJointPose(int joint, Transform3d offset) {
		if (joint < 1 || joint > m_jointCount) { throw new IllegalArgumentException("Invalid joint number " + joint); }

		// Start with the base pose.
		Pose3d pose = m_basePose.get();

		Rotation3d[] rotations = m_jointAngles.get();
		Double[] extensions = m_extensionSupplier.get();

		// Chain transformations for joints 1 up to the specified joint.
		// Note: array indexing is 0-indexed.
		for (int i = 0; i < joint; i++) {
			double length = m_segmentLengths[i] + extensions[i];

			pose = pose.transformBy(new Transform3d(new Translation3d(length, 0, 0), rotations[i]));
		}

		return pose.transformBy(offset);
	}

	/**
	 * Returns the pose of the specified joint (1-indexed). Joint 1 is the first joint (i.e. the
	 * transformation after the base, including the first segment's rotation and translation).
	 *
	 * @param joint The joint number. (1-indexed)
	 * @return The pose of that joint, computed using forward kinematics.
	 * @throws IllegalArgumentException if the joint number is invalid.
	 */
	public Pose3d getJointPose(int joint) {
		return getJointPose(joint, new Transform3d());
	}

	/**
	 * Updates the Network Table outputs. Data is stored in the path as a Pose3d:
	 * <p>
	 * <code>Simulation/{SYSTEM_NAME}Arm/Joint{I}/Pose</code>
	 *
	 * @param offset The offset to apply to each joint.
	 */
	public void update(Transform3d offset) {
		for (int j = 1; j <= m_jointCount; j++) {
			Logger.recordOutput("Simulation/" + m_name + "Arm/Joint" + j + "/Pose", getJointPose(j, offset));
		}
	}

	/**
	 * Updates the Network Table outputs. Data is stored in the path as a Pose3d:
	 * <p>
	 * <code>Simulation/{SYSTEM_NAME}Arm/Joint{I}/Pose</code>
	 *
	 * @param offsets The offsets to apply to each joint.
	 */
	public void update(Transform3d[] offsets) {
		for (int j = 1; j <= m_jointCount; j++) {
			Logger.recordOutput("Simulation/" + m_name + "Arm/Joint" + j + "/Pose", getJointPose(j, offsets[j - 1]));
		}
	}

	/**
	 * Updates the Network Table outputs. Data is stored in the path as a Pose3d:
	 * <p>
	 * <code>Simulation/{SYSTEM_NAME}Arm/Joint{I}/Pose</code>
	 */
	public void update() {
		for (int j = 1; j <= m_jointCount; j++) {
			Logger.recordOutput("Simulation/" + m_name + "Arm/Joint" + j + "/Pose", getJointPose(j));
		}
	}

	/**
	 * Attaches a mechanism to end of the arm.
	 *
	 * @param offset The offset from the end, uses a supplier because attached mechanisms generally
	 *               move or rotate. Generally used to supply rotational offsets.
	 * @return The pose supplier.
	 */
	public Supplier<Pose3d> attachMechanism(Supplier<Transform3d> offset) {
		return () -> computeEndEffectorPose(offset.get());
	}
}
