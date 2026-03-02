package com.bobcats.lib.subsystem.vision.command;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;

import com.bobcats.lib.container.PIDConstants;
import com.bobcats.lib.subsystem.vision.LibVisionSubsystem;
import com.bobcats.lib.utils.PoseUtils;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Optional;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.stream.Collectors;

/**
 * Aligns the robot to the given tag from a birdseye 2D view.
 *
 * <p>
 * Note: Uses <b>Meters</b> and <b>Degrees</b> for PID values.
 *
 * <p>
 * Example usage:
 *
 * <pre>
 * <code>
 * Commands.defer(() -> new AlignToTagCommand2D( // Defer to always reconstruct the command
 *     m_DriveSubsystem::getPose, // Field-relative pose
 *     m_DriveSubsystem::driveFieldOriented, // Field-oriented drive
 *     AlignToTagCommand2D.getClosestTagId(m_DriveSubsystem.getPose(), // Get the nearest Reef tag
 *         6, 7, 8, 9, 10, 11, // Red reef
 *         17, 18, 19, 20, 21, 22 // Blue reef
 *     ),
 *     VisionConstants.kAlignSidewaysOffset, // Offset
 *     VisionConstants.kAlignReefDistance, // Distance
 *     0.9, // Chassis length
 *     new PIDConstants(3), // Translational PID
 *     new PIDConstants(2, 0.01, 0.007), // Angular PID
 *     m_DriveSubsystem), Set.of(m_DriveSubsystem) // Command requirements
 *  );
 * </code> </pre>
 */
public class AlignToTagCommand2D extends Command {

	// Static Config Variables //
	/**
	 * Whether details such as the PID error and setpoint should be logged.
	 */
	public static boolean kDoDetailedDebug = false;

	/**
	 * The default translational tolerance in meters. Defaults to 0.5 meters.
	 */
	public static double kDefaultTranslationalToleranceMeters = Meters.convertFrom(0.5, Centimeters);

	/**
	 * The default angular tolerance in meters. Defaults to 1 degree.
	 */
	public static double kDefaultAngularToleranceDegrees = 1;

	/**
	 * Error counts threshold for when to stop the command. If the error count exceeds this value,
	 * the command will stop and send a warning via DS.
	 */
	public static int kErrorCountThreshold = 15;

	// Pose Data //
	private Pose2d m_pose;
	private Supplier<Pose2d> m_supplier;

	// PID Controllers //
	private PIDController m_xPID, m_yPID, m_aPID;

	// Timer //
	private final Timer m_timer = new Timer();
	private double m_maxTimer = 5;

	private Consumer<ChassisSpeeds> m_consumer;

	// Tag Data //
	private Supplier<Integer> m_tagId = () -> 0;
	private boolean m_updateSetpointsOnce = false;
	private int m_errorCounts = 0;

	private boolean m_doDebug = true;

	// Offsets //
	private double m_alignDistanceMeters, m_horizontalOffsetMeters, m_txOffsetDegrees;

	/**
	 * Constructs a new AlignToTagCommand2D.
	 *
	 * @param poseSupplier           The field-relative robot pose supplier.
	 * @param chassisSpeedsConsumer  The chassis speeds consumer.
	 * @param tagId                  The tag ID to align to.
	 * @param horizontalOffsetMeters The horizontal offset from the tag in meters.
	 * @param alignDistanceMeters    The perpendicular offset from the tag in meters.
	 * @param chassisLengthMeters    The length of the chassis when looked at from the side while
	 *                               aligning in meters.
	 * @param translationalConstants The translational PID constants.
	 * @param angularConstants       The angular PID constants.
	 * @param drive                  The drive subsystem for command requirements.
	 */
	public AlignToTagCommand2D(Supplier<Pose2d> poseSupplier, Consumer<ChassisSpeeds> chassisSpeedsConsumer, int tagId,
			double horizontalOffsetMeters, double alignDistanceMeters, double chassisLengthMeters,
			PIDConstants translationalConstants, PIDConstants angularConstants, Subsystem drive) {
		this(poseSupplier, chassisSpeedsConsumer, tagId, horizontalOffsetMeters, alignDistanceMeters, 0,
				chassisLengthMeters, translationalConstants, angularConstants, drive);
		m_updateSetpointsOnce = true;
	}

	/**
	 * Constructs a new AlignToTagCommand2D.
	 *
	 * @param poseSupplier           The field-relative robot pose supplier.
	 * @param chassisSpeedsConsumer  The chassis speeds consumer.
	 * @param tagId                  The tag ID to align to.
	 * @param horizontalOffsetMeters The horizontal offset from the tag in meters.
	 * @param alignDistanceMeters    The perpendicular offset from the tag in meters.
	 * @param txOffsetDegrees        The angular offset between the tag in degrees. (Right is
	 *                               positive)
	 * @param chassisLengthMeters    The length of the chassis when looked at from the side while
	 *                               aligning in meters.
	 * @param translationalConstants The translational PID constants.
	 * @param angularConstants       The angular PID constants.
	 * @param drive                  The drive subsystem for command requirements.
	 */
	public AlignToTagCommand2D(Supplier<Pose2d> poseSupplier, Consumer<ChassisSpeeds> chassisSpeedsConsumer, int tagId,
			double horizontalOffsetMeters, double alignDistanceMeters, double txOffsetDegrees,
			double chassisLengthMeters, PIDConstants translationalConstants, PIDConstants angularConstants,
			Subsystem drive) {
		this(poseSupplier, chassisSpeedsConsumer, () -> tagId, horizontalOffsetMeters, alignDistanceMeters,
				txOffsetDegrees, chassisLengthMeters, translationalConstants, angularConstants, drive);
		m_updateSetpointsOnce = true;
	}

	/**
	 * Constructs a new AlignToTagCommand2D.
	 *
	 * @param poseSupplier           The field-relative robot pose supplier.
	 * @param chassisSpeedsConsumer  The chassis speeds consumer.
	 * @param tagId                  The supplier of the tag ID to align to.
	 * @param horizontalOffsetMeters The horizontal offset from the tag in meters.
	 * @param alignDistanceMeters    The perpendicular offset from the tag in meters.
	 * @param chassisLengthMeters    The length of the chassis when looked at from the side while
	 *                               aligning in meters.
	 * @param translationalConstants The translational PID constants.
	 * @param angularConstants       The angular PID constants.
	 * @param drive                  The drive subsystem for command requirements.
	 */
	public AlignToTagCommand2D(Supplier<Pose2d> poseSupplier, Consumer<ChassisSpeeds> chassisSpeedsConsumer,
			Supplier<Integer> tagId, double horizontalOffsetMeters, double alignDistanceMeters,
			double chassisLengthMeters, PIDConstants translationalConstants, PIDConstants angularConstants,
			Subsystem drive) {

		this(poseSupplier, chassisSpeedsConsumer, tagId, horizontalOffsetMeters, alignDistanceMeters, 0,
				chassisLengthMeters, translationalConstants, angularConstants, drive);
	}

	/**
	 * Constructs a new AlignToTagCommand2D.
	 *
	 * @param poseSupplier           The field-relative robot pose supplier.
	 * @param chassisSpeedsConsumer  The chassis speeds consumer.
	 * @param tagId                  The supplier of the tag ID to align to.
	 * @param horizontalOffsetMeters The horizontal offset from the tag in meters.
	 * @param alignDistanceMeters    The perpendicular offset from the tag in meters.
	 * @param txOffsetDegrees        The angular offset between the tag in degrees. (Right is
	 *                               positive)
	 * @param chassisLengthMeters    The length of the chassis when looked at from the side while
	 *                               aligning in meters.
	 * @param translationalConstants The translational PID constants.
	 * @param angularConstants       The angular PID constants.
	 * @param drive                  The drive subsystem for command requirements.
	 */
	public AlignToTagCommand2D(Supplier<Pose2d> poseSupplier, Consumer<ChassisSpeeds> chassisSpeedsConsumer,
			Supplier<Integer> tagId, double horizontalOffsetMeters, double alignDistanceMeters, double txOffsetDegrees,
			double chassisLengthMeters, PIDConstants translationalConstants, PIDConstants angularConstants,
			Subsystem drive) {
		m_supplier = poseSupplier;
		m_consumer = chassisSpeedsConsumer;

		m_tagId = tagId;

		m_alignDistanceMeters = alignDistanceMeters + chassisLengthMeters / 2;
		m_horizontalOffsetMeters = -horizontalOffsetMeters;

		m_txOffsetDegrees = txOffsetDegrees;

		// Initialize the PID cntrollers
		m_xPID = new PIDController(translationalConstants.kP, translationalConstants.kI, translationalConstants.kD);
		m_yPID = new PIDController(translationalConstants.kP, translationalConstants.kI, translationalConstants.kD);
		m_aPID = new PIDController(angularConstants.kP, angularConstants.kI, angularConstants.kD);

		// Set the default tolerances
		m_xPID.setTolerance(kDefaultTranslationalToleranceMeters);
		m_yPID.setTolerance(kDefaultTranslationalToleranceMeters);
		m_aPID.setTolerance(kDefaultAngularToleranceDegrees);

		// Angle boundaries
		m_aPID.enableContinuousInput(-180, 180);

		// Set the integral zones
		m_xPID.setIZone(translationalConstants.iZone);
		m_yPID.setIZone(translationalConstants.iZone);
		m_aPID.setIZone(angularConstants.iZone);
		addRequirements(drive);
	}

	@Override
	public void initialize() {
		if (m_doDebug) System.out.println("Starting AlignToTag2D...");

		// Reset the controllers
		m_xPID.reset();
		m_yPID.reset();
		m_aPID.reset();

		m_errorCounts = 0;

		updateSetpoints();

		// Reset and start the timer
		m_timer.restart();
	}

	@Override
	public void execute() {
		// Update the setpoints if needed
		if (!m_updateSetpointsOnce) updateSetpoints();

		// Compute the velocities via the PID controllers
		Pose2d botPose = m_supplier.get();
		ChassisSpeeds speeds = new ChassisSpeeds(m_xPID.calculate(botPose.getX()), m_yPID.calculate(botPose.getY()),
				Math.toRadians(m_aPID.calculate(botPose.getRotation().getDegrees())));
		// Feed the velocities into the drivetrain
		m_consumer.accept(speeds);

		// Debugging
		if (kDoDetailedDebug && m_doDebug) System.out.printf(
				"AlignToTag2D debug: eX: %.2f eY: %.2f eA: %.2f | Setpoint X: %.2f Y: %.2f A: %.2f | Timer: %.2f\n",
				m_xPID.getError(), m_yPID.getError(), m_aPID.getError(), m_xPID.getSetpoint(), m_yPID.getSetpoint(),
				m_aPID.getSetpoint(), m_timer.get());
	}

	@Override
	public boolean isFinished() {
		// End conditions:
		// * Timer exceeded max time
		// * Setpoint has been reached
		// * Error count threshold exceeded

		return m_xPID.atSetpoint() && m_yPID.atSetpoint() && m_aPID.atSetpoint() || m_timer.hasElapsed(m_maxTimer);
	}

	@Override
	public void end(boolean interrupted) {
		// Halt everything
		m_timer.stop();
		m_consumer.accept(new ChassisSpeeds(0, 0, 0));

		// Debug messages
		if (interrupted && m_doDebug) {
			System.out.printf("AlignToTag2D has been interrupted. Time (s): %.2f\n", m_timer.get());
		} else if (m_doDebug) {
			System.out.printf("AlignToTag2D has finished successfully. Time (s): %.2f\n", m_timer.get());
		}
	}

	/**
	 * Sets the tolerance of the translational (x, y) PID controllers. Defaults to 0.02 meters = 2
	 * cm.
	 *
	 * @param toleranceMeters The tolerance in meters.
	 * @return The instance for chaining.
	 */
	public AlignToTagCommand2D setTranslationalTolerance(double toleranceMeters) {
		m_xPID.setTolerance(toleranceMeters);
		m_yPID.setTolerance(toleranceMeters);
		return this;
	}

	/**
	 * Sets the tolerance of the angular PID controller. Defaults to 1 degree.
	 *
	 * @param toleranceDegrees The tolerance in degrees.
	 * @return The instance for chaining.
	 */
	public AlignToTagCommand2D setAngularTolerance(double toleranceDegrees) {
		m_aPID.setTolerance(toleranceDegrees);
		return this;
	}

	/**
	 * Sets the maximum duration of the timer. Defaults to 5 seconds.
	 *
	 * @param maxSeconds The maximum duration in seconds.
	 * @return The instance for chaining.
	 */
	public AlignToTagCommand2D setMaxTimer(double maxSeconds) {
		m_maxTimer = maxSeconds;
		return this;
	}

	/**
	 * Sets whether basic debugging is enabled or not. Defaults to true.
	 *
	 * @param enabled True if enabled.
	 * @return The instance for chaining.
	 */
	public AlignToTagCommand2D setDoDebug(boolean enabled) {
		m_doDebug = enabled;
		return this;
	}

	/**
	 * Finds the ID of the AprilTag whose pose is closest to the given robotPose. If no validTagIds
	 * are passed, all tags are considered. Return -1 if no valid tags are found.
	 *
	 * @param robotPose   The field-relative robot pose.
	 * @param validTagIds (Nullable) Optional filter of allowed tag IDs.
	 * @return The closest tag ID.
	 */
	public static int getClosestTagId(Pose2d robotPose, int... validTagIds) {
		// If filter supplied, turn into a Set for O(1) lookups
		Set<Integer> filter = (validTagIds == null || validTagIds.length == 0) ? null
				: Arrays.stream(validTagIds).boxed().collect(Collectors.toSet());

		Optional<AprilTag> best = LibVisionSubsystem.kLayout.getTags()
				.stream()
				// if filter!=null, only keep tags in it
				.filter(tag -> filter == null || filter.contains(tag.ID))
				// pick the one with minimum distance
				.min(Comparator.comparingDouble(
						tag -> tag.pose.toPose2d().getTranslation().getDistance(robotPose.getTranslation())));
		return best.map(tag -> tag.ID).orElse(-1); // -1 if no tags match
	}

	// Private methods //

	private void updateSetpoints() {
		var pose = LibVisionSubsystem.kLayout.getTagPose(m_tagId.get());
		if (!pose.isPresent()) {
			m_errorCounts++;
			DriverStation.reportWarning(
					"AlignToTag2D::updateSetpoints: invalid tag ID '" + m_tagId.get() + "', counts " + m_errorCounts,
					true);
			return;
		} else {
			m_errorCounts--;
			if (m_errorCounts < 0) m_errorCounts = 0;
		}

		// Error count threshold
		if (m_errorCounts > kErrorCountThreshold) {
			DriverStation.reportWarning("AlignToTag2D has been interrupted due to too many tag errors", true);
			m_errorCounts = 0;
			cancel();
			return;
		}

		// Offset the pose
		m_pose = PoseUtils.offsetSideways(PoseUtils.offsetPerpendicular(pose.get().toPose2d(), m_alignDistanceMeters),
				m_horizontalOffsetMeters);

		// Apply setpoints
		m_xPID.setSetpoint(m_pose.getX());
		m_yPID.setSetpoint(m_pose.getY());
		// Add 180 to invert the look of the robot to face towards the tag
		m_aPID.setSetpoint(180 + m_pose.getRotation().getDegrees() - m_txOffsetDegrees);
	}
}
