package com.bobcats.lib.ascope.linear;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Simulates a cascade elevator and calculates stage positions, publishing them to Network
 * Tables (via Akit Logger) for visualization in AdvantageScope.
 *
 * <p>
 * Model: If N stages are present (N >= 1) and the elevator's "raw" height is h, then the i'th
 * stage gets a fraction of h: ((N - i + 1) / N) * h.
 */
public class CascadeAscopeDisplay {
	private final int m_stages;
	private final double m_offset;
	private final Supplier<Double> m_heightMeters;
	private final String m_name;
	private final Supplier<Rotation3d> m_angle;
	private final double m_elevatorMaxHeight;

	/**
	 * Constructs a new CascadeAscopeDisplay.
	 *
	 * @param name                    The name of the system.
	 * @param stages                  The number of stages in the elevator (>= 1).
	 * @param offsetMeters            A baseline offset (in meters). Often used if the zero point
	 *                                of the height sensor isn't actually the bottom of the
	 *                                elevator.
	 * @param heightSupplierMeters    The supplier returning the elevator's current raw height (in
	 *                                meters).
	 * @param angleRadians            The tilt of the elevator as a Rotation3d (use
	 *                                Rotation3d.kZero for upright).
	 * @param elevatorMaxHeightMeters The physical max height (in meters) of the elevator.
	 */
	public CascadeAscopeDisplay(String name, int stages, double offsetMeters, Supplier<Double> heightSupplierMeters,
			Supplier<Rotation3d> angleRadians, double elevatorMaxHeightMeters) {
		if (stages < 1) { throw new IllegalArgumentException("Stages must be at least 1"); }
		m_name = name;
		m_stages = stages;
		m_offset = offsetMeters;
		m_heightMeters = heightSupplierMeters;
		m_angle = angleRadians;
		m_elevatorMaxHeight = elevatorMaxHeightMeters;
	}

	/**
	 * Constructs a new CascadeAscopeDisplay with an upright elevator (no tilt).
	 *
	 * @param name                    The name of the system.
	 * @param stages                  The number of stages in the elevator (>= 1).
	 * @param offsetMeters            A baseline offset (in meters). Often used if the zero point
	 *                                of the height sensor isn't actually the bottom of the
	 *                                elevator.
	 * @param heightSupplierMeters    The supplier returning the elevator's current raw height (in
	 *                                meters).
	 * @param elevatorMaxHeightMeters The physical max height (in meters) of the elevator.
	 */
	public CascadeAscopeDisplay(String name, int stages, double offsetMeters, Supplier<Double> heightSupplierMeters,
			double elevatorMaxHeightMeters) {
		this(name, stages, offsetMeters, heightSupplierMeters, () -> Rotation3d.kZero, elevatorMaxHeightMeters);
	}

	/**
	 * Updates the Network Table outputs. Data is stored in the path as a Pose3d:
	 * <p>
	 * <code>Simulation/{SYSTEM_NAME}Elevator/Stage{I}/Pose</code>
	 */
	public void update() {
		for (int i = 1; i <= m_stages; i++) {
			Logger.recordOutput("Simulation/" + m_name + "Elevator/Stage" + i + "/Pose", getStagePose(i));
		}
	}

	/**
	 * Returns the pose of the specified stage.
	 *
	 * @param stage The stage number (1 = carriage).
	 * @return The AdvantageScope zero-position relative pose.
	 */
	public Pose3d getStagePose(int stage) {
		if (stage < 1 || stage > m_stages) {
			// Invalid stage index
			return new Pose3d();
		}

		// Reverse the stage order so stage=1 is the top (largest fraction).
		int reversedStage = m_stages - stage + 1;

		// Clamp raw height to avoid overshooting once offset is added.
		double rawHeight = MathUtil.clamp(m_heightMeters.get() + m_offset, 0,
				Math.max(0.0, m_elevatorMaxHeight + m_offset));

		// Compute fraction for this stage (top stage gets fraction=1.0, bottom stage
		// fraction=1/m_stages).
		double fraction = (double) reversedStage / (double) m_stages;
		double fractionHeight = fraction * rawHeight;

		// Add offset as a baseline.
		double zHeight = fractionHeight; // + m_offset;

		// Final clamp to ensure we never exceed physical max.
		zHeight = MathUtil.clamp(zHeight, 0, m_elevatorMaxHeight);

		// Create the Pose3d at zHeight, rotated by m_angle.
		return new Pose3d().transformBy(
				new Transform3d(new Translation3d(0.0, 0.0, zHeight).rotateBy(m_angle.get()), m_angle.get()));
	}

	/**
	 * Attaches a mechanism to the carriage.
	 *
	 * @param offset The offset from the carriage, uses a supplier because attached mechanisms
	 *               generally move or rotate. Generally used to supply rotational offsets.
	 * @return The offset pose supplier from the carriage.
	 */
	public Supplier<Pose3d> attachMechanism(Supplier<Transform3d> offset) {
		return () -> getStagePose(1).transformBy(offset.get());
	}
}
