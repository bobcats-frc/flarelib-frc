package com.bobcats.lib.ascope.linear;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Simulates a continuous elevator and calculates stage positions, publishing them to Network
 * Tables (via Akit Logger) for visualization in AdvantageScope.
 *
 * <p>
 * First, the carriage moves, but this doesn't affect any other stages. Then, stage 2 raises
 * and moves all the higer stages and the carriage. This goes on for all other stages. And this
 * assumes that the carriage is attached to the last stage of the elevator.
 *
 * <p>
 * NOTE: The height is based off the carriage.
 *
 * <p>
 * <i>Automatically accounts for the offset provided.</i>
 */
public class ContinuousAscopeDisplay {
	private final int m_totalStages;
	private final double m_maxHeight;
	private final Supplier<Double> m_heightSupplier;
	private final Supplier<Rotation3d> m_angle;
	private final String m_name;

	private final double[] m_stageMaxHeights;

	// The physical order of stages.
	private final int[] m_physicalOrder;

	// The pose offset applied to the supplied pose. Useful for situations where a mechanism pose
	// is provided instead of a carriage pose.
	private final double m_offset;

	/**
	 * Constructs a new ContinuousAscopeDisplay.
	 *
	 * @param name            The name of the system.
	 * @param totalStages     The amount of stages present. The first stage is the carriage.
	 * @param maxHeightMeters The maximum reachable height, in meters.
	 * @param offset          The pose offset applied to the supplied pose, in meters. Used when a
	 *                        mechanism pose is provided instead of a carriage pose.
	 * @param heightSupplier  The elevator height supplier.
	 * @param angle           The elevator angle supplier.
	 */
	public ContinuousAscopeDisplay(String name, int totalStages, double maxHeightMeters, double offset,
			Supplier<Double> heightSupplier, Supplier<Rotation3d> angle) {
		if (totalStages < 1) throw new IllegalArgumentException("At least 1 stage required");
		m_name = name;
		m_totalStages = totalStages;
		m_maxHeight = maxHeightMeters;
		m_heightSupplier = heightSupplier;
		m_angle = angle;

		// Allocate stageMaxHeights.
		// Extensions are split equally among stages 1->N.
		m_stageMaxHeights = new double[totalStages];
		for (int i = 0; i < totalStages; i++) { m_stageMaxHeights[i] = maxHeightMeters / totalStages; }

		// Build the physical order array: bottom is Stage 2, then 3->N, top is Stage 1.
		// For example, if totalStages=4, this yields [2, 3, 4, 1].
		m_physicalOrder = new int[m_totalStages];
		for (int i = 2; i <= m_totalStages; i++) {
			m_physicalOrder[i - 2] = i; // fill from index 0->(N-2)
		}
		m_physicalOrder[m_totalStages - 1] = 1; // put stage 1 at the very end (the top)

		m_offset = offset;
	}

	/**
	 * Constructs a new ContinuousAscopeDisplay.
	 *
	 * @param name            The name of the system.
	 * @param totalStages     The amount of stages present. The first stage is the carriage.
	 * @param maxHeightMeters The maximum reachable height, in meters.
	 * @param offset          The pose offset applied to the supplied pose, in meters. Used when a
	 *                        mechanism pose is provided instead of a carriage pose.
	 * @param heightSupplier  The elevator height supplier.
	 * @param angle           The elevator angle supplier.
	 * @param stageLengths    The length of each stage in ascending order, starting from stage 1.
	 */
	public ContinuousAscopeDisplay(String name, int totalStages, double maxHeightMeters, double offset,
			Supplier<Double> heightSupplier, Supplier<Rotation3d> angle, double[] stageLengths) {
		if (totalStages < 1) { throw new IllegalArgumentException("At least 1 stage required"); }
		m_name = name;
		m_totalStages = totalStages;
		m_maxHeight = maxHeightMeters;
		m_heightSupplier = heightSupplier;
		m_angle = angle;

		// Allocate stageMaxHeights.
		m_stageMaxHeights = stageLengths;

		// Build the physical order array: bottom is Stage 2, then 3->N, top is Stage 1.
		// For example, if totalStages=4, this yields [2, 3, 4, 1].
		m_physicalOrder = new int[m_totalStages];
		for (int i = 2; i <= m_totalStages; i++) {
			m_physicalOrder[i - 2] = i; // fill from index 0->(N-2)
		}
		m_physicalOrder[m_totalStages - 1] = 1; // put stage 1 at the very end (the top)
		m_offset = offset;
	}

	/**
	 * Updates the Network Table outputs. Data is stored in the path as a Pose3d:
	 * <p>
	 * <code>Simulation/{SYSTEM_NAME}Elevator/Stage{I}/Pose</code>
	 */
	public void update() {
		for (int stage = 1; stage <= m_totalStages; stage++) {
			Logger.recordOutput("Simulation/" + m_name + "Elevator/Stage" + stage + "/Pose", getStagePose(stage));
		}
	}

	/**
	 * Updates the Network Table outputs. Data is stored in the path as a Pose3d:
	 * <p>
	 * <code>Simulation/{SYSTEM_NAME}Elevator/Stage{I}/Pose</code>
	 *
	 * @param additionalOffset The additional offset to apply to each stage.
	 */
	public void update(Transform3d additionalOffset) {
		for (int stage = 1; stage <= m_totalStages; stage++) {
			Logger.recordOutput("Simulation/" + m_name + "Elevator/Stage" + stage + "/Pose",
					getStagePose(stage).transformBy(additionalOffset));
		}
	}

	/**
	 * Returns the 3D pose of the given stage (1-indexed).
	 *
	 * @param stage The stage number, 1-indexed.
	 * @return The AdvantageScope zero-position relative pose.
	 */
	public Pose3d getStagePose(int stage) {
		// Figure out how much each stage has extended.
		double total = MathUtil.clamp(m_heightSupplier.get() + m_offset, 0, m_maxHeight);

		// Check to allow for 1-stage elevators:
		if (m_totalStages == 1) return createPose(total);

		double[] ext = computeExtensions(total);
		// ext[0] = how far Stage 1 extended,
		// ext[1] = how far Stage 2 extended,
		// etc.

		// Sum up the relevant extensions in physical order from the bottom up until we reach 'stage'.
		// Because each stage that is below this stage physically lifts it.
		double z = 0.0;
		for (int i = 0; i < m_physicalOrder.length; i++) {
			int st = m_physicalOrder[i];
			z += ext[st - 1]; // add that stage's extension
			if (st == stage) {
				// we've reached the requested stage, so stop summing
				break;
			}
		}

		// Construct a Pose3d with this final vertical offset.
		return createPose(z);
	}

	/**
	 * Compute how far each stage extends given the total requested height.
	 *
	 * <p>
	 * Usage order:
	 * <p>
	 * - Stage 1 extends first, up to m_stageMaxHeights[0].
	 * <p>
	 * - Then Stage 2 extends, up to m_stageMaxHeights[1].
	 * <p>
	 * - Then Stage 3, etc., until leftover is 0.
	 *
	 * @param total The total height of the elevator.
	 * @return The extensions.
	 */
	private double[] computeExtensions(double total) {
		double[] result = new double[m_totalStages];
		double leftover = total;

		// Allocate to each stage in order 1->N
		for (int i = 0; i < m_totalStages; i++) {
			double allocated = Math.min(leftover, m_stageMaxHeights[i]);
			result[i] = allocated;
			leftover -= allocated;
			if (leftover <= 0) { break; }
		}
		return result;
	}

	/**
	 * Helper to build the final Pose3d from a vertical offset Z.
	 *
	 * @param zHeight The height, in meters.
	 * @return The rotation applied pose.
	 */
	private Pose3d createPose(double zHeight) {
		return new Pose3d()
				.transformBy(new Transform3d(new Translation3d(0, 0, zHeight).rotateBy(m_angle.get()), m_angle.get()));
	}

	/**
	 * Attaches a mechanism to the carriage.
	 *
	 * @param offset The offset from the carriage, uses a supplier because attached mechanisms
	 *               generally move or rotate. Generally used to supply rotational offsets.
	 * @return The pose supplier.
	 */
	public Supplier<Pose3d> attachMechanism(Supplier<Transform3d> offset) {
		return () -> getStagePose(1).transformBy(offset.get());
	}
}
