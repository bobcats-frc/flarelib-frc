package com.bobcats.lib.auto;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.bobcats.lib.utils.AllianceUtil;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.Vector2;

/**
 * A utility class for working with line segments, and optimizing on-the-fly robot
 * trajectories.
 *
 * <p>
 * <b>Usage Example</b>: The 2024 Crescendo season, auto-shoot: to get the robot near the
 * speaker (auto-range) shot zone, while avoiding the stage (triangle denoted with 3 line
 * segments).
 *
 * <p>
 * <b>Note</b>: Uses field-relative coordinates, not alliance-relative. Make sure to flip the
 * poses accordingly, or the robot may crash into an obstacle.
 *
 * <p>
 * <b>Note</b>: For the methods that use {@link AutoBuilder}, the AutoBuilder must be
 * configured, otherwise an exception will be thrown by AutoBuilder.
 *
 * <p>
 * <b>Note</b>: There may be better alternatives for picking simple paths if there aren't too
 * many parameters, as this class is computationally expensive. Then, instead use PP's
 * {@link AutoBuilder} to go to pre-defined points.
 */
public class LineTrajectoryUtils {

	private static final double kLookAtMinDist = 0.1; // Units: meters (m)

	/**
	 * The period to check for re-ranging in the method auto ranging methods. Setting this too low
	 * may get your robot stuck near the edge boundary of the in-range convex. Defaults to 0.7
	 * seconds.
	 */
	public static double kRerangeCheckPeriodSeconds = 0.7;

	/** No instantiation for utility classes. */
	private LineTrajectoryUtils() {
		throw new UnsupportedOperationException("Can't instantiate utility class LineTrajectoryUtils");
	}

	/**
	 * Returns whether the given lines intersect once.
	 *
	 * @param line1 The first line.
	 * @param line2 The second line.
	 * @return True if the lines intersect once, false otherwise.
	 * @see <a href="https://stackoverflow.com/questions/3838329/">StackOverflow answer</a>
	 */
	public static boolean doesIntersectOnce(Line line1, Line line2) {
		return isCCW(line1.start, line2.start, line2.end) != isCCW(line1.end, line2.start, line2.end)
				&& isCCW(line1.start, line1.end, line2.start) != isCCW(line1.start, line1.end, line2.end);
	}

	/**
	 * Optimizes a line to go past all the lines in the given list, avoiding intersections with
	 * them.
	 *
	 * <p>
	 * <b>Usage Example</b>: The 2024 Crescendo season, auto-shoot.
	 *
	 * @param startingPoint        The starting point.
	 * @param linesToAvoid         The list of lines to avoid intersecting with.
	 * @param targetLine           The target line to optimize towards.
	 * @param steps                The max number of steps to take in the optimization process.
	 * @param distanceToJustAccept The minimum distance to accept a line as the shortest, to
	 *                             minimize optimization steps.
	 * @return A new line that goes from the starting point to the target line, avoiding
	 *         intersections with the lines in the list. Has the minimum possible distance.
	 * @see #optimizeUntilNoIntersect_robotSize(Translation2d, List, Line, int, double, double,
	 *      double)
	 */
	public static OptimizedLine optimizeUntilNoIntersect(Translation2d startingPoint, List<Line> linesToAvoid,
			Line targetLine, int steps, double distanceToJustAccept) {
		Translation2d A = startingPoint;
		Translation2d B = targetLine.start;
		Translation2d C = targetLine.end;

		double minDistance = Double.MAX_VALUE;
		OptimizedLine line = new OptimizedLine(new Line(startingPoint, targetLine.end.plus(targetLine.start).div(2)),
				false);
		for (int i = 0; i <= steps; i++) {
			double t = i / (double) steps;
			// Interpolate along the line: point = B + t*(C - B)
			Translation2d candidateEnd = B.plus(C.minus(B).times(t));
			Line candidateShot = new Line(A, candidateEnd);

			boolean isOk = true;
			// Check for collision with all lines to avoid
			// If any line intersects with the candidate shot, we discard it
			for (Line avoid : linesToAvoid)
				if (doesIntersectOnce(candidateShot, avoid)) { isOk = false; break; }

			// If the candidate shot does not intersect with any line, we return it
			if (isOk) {
				double dist = candidateShot.start.getDistance(candidateShot.end);
				if (dist < distanceToJustAccept) return new OptimizedLine(candidateShot, true);
				if (dist < minDistance) line = new OptimizedLine(candidateShot, true);

				minDistance = Math.min(minDistance, dist);
			}
		}

		return line;
	}

	/**
	 * Optimizes a line to go past all the lines in the given list, avoiding intersections with
	 * them and starting at the given line. Assumes the robot is a point with no size. Keep in mind
	 * that this may give non-optimal results if the point provided is between the lines or past
	 * the target line.
	 *
	 * <p>
	 * <b>Warning</b>: This method is computationally expensive, may cause lag if called
	 * frequently.
	 *
	 * <p>
	 * <b>Usage Example</b>: The 2024 Crescendo season, auto-shoot.
	 *
	 * @param startingLine           The line to start from, which will be optimized.
	 * @param linesToAvoid           The list of lines to avoid intersecting with.
	 * @param targetLine             The target line to optimize towards.
	 * @param stepsAlongStartingLine The number of steps to take along the starting line.
	 * @param stepsPerIntersectCheck The number of steps to take when checking for intersections.
	 * @param botPoseForDriveOpt     (Nullable) The robot's pose for drive optimization, if
	 *                               applicable. Uses a different alogrithm to minimze total
	 *                               driving distance instead of the line segment between the two
	 *                               lines.
	 * @param distanceToJustAccept   The minimum distance to accept a line as the shortest. Also
	 *                               used for the total driving distance/the line segment distance.
	 * @return A new line that goes from the starting line to the target line, avoiding
	 *         intersections with the lines in the list. Has the minimum possible distance (or the
	 *         min driving distance based on the bot pose parameter).
	 * @see #optimizeUntilNoIntersect_robotSize(Line, List, Line, int, int, Translation2d, double,
	 *      double, double)
	 */
	public static OptimizedLine optimizeUntilNoIntersect(Line startingLine, List<Line> linesToAvoid, Line targetLine,
			int stepsAlongStartingLine, int stepsPerIntersectCheck, Translation2d botPoseForDriveOpt,
			double distanceToJustAccept) {
		double minDist = Double.POSITIVE_INFINITY;
		OptimizedLine bestResult = new OptimizedLine(null, false);
		for (int i = 0; i < stepsAlongStartingLine; i++) {
			double t = i / (double) stepsAlongStartingLine;
			// Interpolate along the starting line: point = start + t*(end - start)
			Translation2d startingPointTest = startingLine.start
					.plus(startingLine.end.minus(startingLine.start).times(t));

			OptimizedLine result = optimizeUntilNoIntersect(startingPointTest, linesToAvoid, targetLine,
					stepsPerIntersectCheck, distanceToJustAccept);
			if (!result.success) continue; // If the optimization failed, skip it

			if (botPoseForDriveOpt != null) {
				// Minimze the total distance to drive the robot
				double distanceDriven = result.optimizedLine.start.getDistance(result.optimizedLine.end)
						+ botPoseForDriveOpt.getDistance(result.optimizedLine.start);
				if (distanceDriven < distanceToJustAccept) return result;
				if (distanceDriven < minDist) { bestResult = result; minDist = distanceDriven; }
			} else {
				// Minimize the distance of the line segment
				double distance = result.optimizedLine.start.getDistance(result.optimizedLine.end);
				if (distance < distanceToJustAccept) return result;
				if (distance < minDist) { bestResult = result; minDist = distance; }
			}
		}

		return bestResult;
	}

	/**
	 * Optimizes a line to go past all the lines in the given list, avoiding intersections with
	 * them. Assumes the robot is a rectangle with the given sizes along the coordinate axes.
	 *
	 * <p>
	 * <b>Usage Example</b>: The 2024 Crescendo season, auto-shoot.
	 *
	 * @param startingPoint        The starting point.
	 * @param linesToAvoid         The list of lines to avoid intersecting with.
	 * @param targetLine           The target line to optimize towards.
	 * @param steps                The max number of steps to take in the optimization process.
	 * @param distanceToJustAccept The minimum distance to accept a line as the shortest, to
	 *                             minimize optimization steps.
	 * @param sizeX                The size of the robot along the X axis, in meters.
	 * @param sizeY                The size of the robot along the Y axis, in meters.
	 * @return A new line that goes from the starting point to the target line, avoiding
	 *         intersections with the lines in the list. Has the minimum possible distance.
	 */
	public static OptimizedLine optimizeUntilNoIntersect_robotSize(Translation2d startingPoint, List<Line> linesToAvoid,
			Line targetLine, int steps, double distanceToJustAccept, double sizeX, double sizeY) {
		Translation2d A = startingPoint;
		Translation2d B = targetLine.start;
		Translation2d C = targetLine.end;

		double minDist = Double.MAX_VALUE;
		OptimizedLine best = new OptimizedLine(new Line(A, B.plus(C).div(2.0)), false);

		for (int i = 0; i <= steps; i++) {
			double t = i / (double) steps;
			Translation2d endPt = B.plus(C.minus(B).times(t));
			Line shot = new Line(A, endPt);

			boolean clear = true;
			for (Line avoid : linesToAvoid) {
				if (doesIntersectOnce(shot, avoid)) { clear = false; break; }
				if (rectCollision(shot, avoid, sizeX / 2, sizeY / 2)) { clear = false; break; }
			}

			if (clear) {
				double d = A.getDistance(endPt);
				if (d < distanceToJustAccept) return new OptimizedLine(shot, true);
				if (d < minDist) { minDist = d; best = new OptimizedLine(shot, true); }
			}
		}

		return best;
	}

	/**
	 * Optimizes a line to go past all the lines in the given list, avoiding intersections with
	 * them and starting at the given line. Assumes the robot is a rectangle with the given sizes
	 * along the coordinate axes. Keep in mind that this may give non-optimal results if the point
	 * provided is between the lines or past the target line. Also keep in mind this only optimizes
	 * linear trajectory, and rotating the robot may result in collisions unless a clearance is
	 * provided.
	 *
	 * <p>
	 * <b>Warning</b>: This method is computationally expensive, may cause lag if called
	 * frequently.
	 *
	 * <p>
	 * <b>Usage Example</b>: The 2024 Crescendo season, auto-shoot.
	 *
	 * @param startingLine           The line to start from, which will be optimized.
	 * @param linesToAvoid           The list of lines to avoid intersecting with.
	 * @param targetLine             The target line to optimize towards.
	 * @param stepsAlongStartingLine The number of steps to take along the starting line.
	 * @param stepsPerIntersectCheck The number of steps to take when checking for intersections.
	 * @param botPoseForDriveOpt     (Nullable) The robot's pose for drive optimization, if
	 *                               applicable. Uses a different alogrithm to minimze total
	 *                               driving distance instead of the line segment between the two
	 *                               lines.
	 * @param distanceToJustAccept   The minimum distance to accept a line as the shortest. Also
	 *                               used for the total driving distance/the line segment distance.
	 * @param sizeX                  The size of the robot along the X axis, in meters.
	 * @param sizeY                  The size of the robot along the Y axis, in meters.
	 * @return A new line that goes from the starting line to the target line, avoiding
	 *         intersections with the lines in the list. Has the minimum possible distance (or the
	 *         min driving distance based on the bot pose parameter).
	 */
	public static OptimizedLine optimizeUntilNoIntersect_robotSize(Line startingLine, List<Line> linesToAvoid,
			Line targetLine, int stepsAlongStartingLine, int stepsPerIntersectCheck, Translation2d botPoseForDriveOpt,
			double distanceToJustAccept, double sizeX, double sizeY) {
		double maxDist = Double.POSITIVE_INFINITY;
		OptimizedLine best = new OptimizedLine(null, false);

		for (int i = 0; i <= stepsAlongStartingLine; i++) {
			double t = i / (double) stepsAlongStartingLine;
			Translation2d sliceStart = startingLine.start.plus(startingLine.end.minus(startingLine.start).times(t));

			OptimizedLine result = optimizeUntilNoIntersect_robotSize(sliceStart, linesToAvoid, targetLine,
					stepsPerIntersectCheck, distanceToJustAccept, sizeX, sizeY);

			if (!result.success()) continue;

			Line shot = result.optimizedLine();
			double dist = shot.start.getDistance(shot.end);
			if (botPoseForDriveOpt != null) dist += botPoseForDriveOpt.getDistance(shot.start);

			if (dist < distanceToJustAccept) return result;
			if (dist < maxDist) { maxDist = dist; best = result; }
		}

		return best;
	}

	/**
	 * Repeatedly tries to auto-range using PathPlanner's {@link AutoBuilder}, until within the
	 * tolerance or optionally when the timeout is exceeded. <b> Just like other methods, this
	 * method is not alliance-aware, and doesn't flip the path automatically. Provide appropriately
	 * flipped coordinates.</b> AutoBuilder must be configured. More accurate compared to
	 * {@link #autoRangeAndLookAt(Supplier, Supplier, PathConstraints, double, Double, double, double)}.
	 * <b>Returns a deferred command.</b> Recommended to bind to a {@link Trigger} instance.
	 *
	 * <p>
	 * This method may take a really long time to run the first time, but will run faster. Still,
	 * this method is relatively <b>performance intensive</b>.
	 *
	 * @param optLine                      The optimized line trajectory. Obtained via the
	 *                                     optimization methods.
	 * @param lookAt                       The rotation to look at, set to robot's current angle to
	 *                                     discard.
	 * @param constraints                  The path constraints.
	 * @param maxTime                      (Nullable) The maximum time to attempt for. Highly
	 *                                     recommended to set a value.
	 * @param toleranceTranslationalMeters The translational tolerance, in meters.
	 * @param angleToleranceDegs           The angular tolerance, in degrees.
	 * @param inRangeArea                  If the robot is in this area, the request is ignored.
	 *                                     Put a null supplier {@code () -> null} to ignore.
	 * @param poseSupplier                 The supplier for the robot's estimated pose.
	 * @param rotateFunc                   The method to rotate the robot to the given rotation,
	 *                                     given the rotation setpoint and current pose. Applies,
	 *                                     then returns the calculated angular velocity. Units
	 *                                     should be rad/s.
	 * @param stopRotation                 The method to stop <b>ONLY</b> the robot's rotation.
	 * @param endAfterRotating             Whether or not to end the command after the rotational
	 *                                     setpoint is reached after pathfind, or to continue the
	 *                                     command until interruption.
	 * @param reRangeWhenOut               Whether to re-range the robot if it ever goes out of
	 *                                     bounds. <b>This may not work as intended if inRangeArea
	 *                                     is a null supplier.</b>
	 * @return The command to pathfind, as a <b>deferred command</b>.
	 * @see #autoRangeAndLookAt_noDefer(OptimizedLine, Rotation2d, PathConstraints, Double, double,
	 *      double, Supplier, Supplier, BiFunction, Runnable, boolean)
	 * @see #autoRangeAndLookAt(Supplier, Supplier, PathConstraints, double, Double, double,
	 *      double)
	 * @see #ofAllianceAwareLine(Translation2d, Translation2d)
	 */
	public static Command autoRangeAndLookAt(Supplier<OptimizedLine> optLine, Supplier<Rotation2d> lookAt,
			PathConstraints constraints, Double maxTime, double toleranceTranslationalMeters, double angleToleranceDegs,
			Supplier<Convex> inRangeArea, Supplier<Pose2d> poseSupplier,
			BiFunction<Rotation2d, Pose2d, AngularVelocity> rotateFunc, Runnable stopRotation,
			BooleanSupplier endAfterRotating, boolean reRangeWhenOut) {
		if (maxTime != null) return new DeferredCommand(() -> AutoBuilder
				// Build pathfind command to get to the translational pose
				.pathfindToPose(new Pose2d(optLine.get().optimizedLine().end(), lookAt.get()), constraints, 0.0)
				// Run until accurate
				.repeatedly()
				// End condition of the pathfind
				.until(() -> {
					Pose2d retPos = poseSupplier.get();
					return retPos.getTranslation()
							.getDistance(optLine.get().optimizedLine().end()) <= toleranceTranslationalMeters;
				})
				// No need to run if in range, just look at target
				.onlyIf(() -> inRangeArea.get() == null || !inRangeArea.get()
						.contains(new Vector2(poseSupplier.get().getX(), poseSupplier.get().getY()))),
				Set.of())
						// Apply rotational override to the pathplanner path
						.alongWith(Commands.runOnce(() -> {
							PPHolonomicDriveController.overrideRotationFeedback(
									() -> rotateFunc.apply(lookAt.get(), poseSupplier.get()).in(RadiansPerSecond));
						}))
						.andThen(Commands.parallel(new DeferredCommand(() -> AutoBuilder
								// Build pathfind command to get to the translational pose
								.pathfindToPose(new Pose2d(optLine.get().optimizedLine().end(), lookAt.get()),
										constraints, 0.0)
								// Run until accurate
								// .repeatedly()
								// End condition of the pathfind
								.until(() -> {
									Pose2d retPos = poseSupplier.get();
									return retPos.getTranslation()
											.getDistance(optLine.get()
													.optimizedLine()
													.end()) <= toleranceTranslationalMeters;
								})
								// Only re-range if outside bounds
								.onlyIf(() -> !inRangeArea.get()
										.contains(new Vector2(poseSupplier.get().getX(), poseSupplier.get().getY())))
								// Enforce minimum check period limit to avoid getting stuck, run periodically
								.alongWith(Commands.waitSeconds(kRerangeCheckPeriodSeconds)), Set.of()).repeatedly()
										// Only run if applicable
										.onlyIf(() -> reRangeWhenOut),
								// Run the rotation command to look at the target, rotate until within tolerance
								new DeferredCommand(
										() -> Commands.run(() -> rotateFunc.apply(lookAt.get(), poseSupplier.get()))
												.until(() -> endAfterRotating.getAsBoolean()
														&& angleWithinTolerance(lookAt.get(),
																poseSupplier.get().getRotation(), angleToleranceDegs)),
										// .finallyDo(PPHolonomicDriveController::clearRotationFeedbackOverride),
										Set.of())))
						.finallyDo(() -> {
							stopRotation.run();
							PPHolonomicDriveController.clearRotationFeedbackOverride();
						})
						.onlyIf(() -> optLine.get() != null && optLine.get().success())
						.withTimeout(maxTime.doubleValue());

		return new DeferredCommand(() -> AutoBuilder
				// Build pathfind command to get to the translational pose
				.pathfindToPose(new Pose2d(optLine.get().optimizedLine().end(), lookAt.get()), constraints, 0.0)
				// Run until accurate
				.repeatedly()
				// End condition of the pathfind
				.until(() -> {
					Pose2d retPos = poseSupplier.get();
					return retPos.getTranslation()
							.getDistance(optLine.get().optimizedLine().end()) <= toleranceTranslationalMeters;
				})
				// No need to run if in range, just look at target
				.onlyIf(() -> inRangeArea.get() == null || !inRangeArea.get()
						.contains(new Vector2(poseSupplier.get().getX(), poseSupplier.get().getY()))),
				Set.of())
						// Apply rotational override to the pathplanner path
						.alongWith(Commands.runOnce(() -> {
							PPHolonomicDriveController.overrideRotationFeedback(
									() -> rotateFunc.apply(lookAt.get(), poseSupplier.get()).in(RadiansPerSecond));
						}))
						.andThen(Commands.parallel(new DeferredCommand(() -> AutoBuilder
								// Build pathfind command to get to the translational pose
								.pathfindToPose(new Pose2d(optLine.get().optimizedLine().end(), lookAt.get()),
										constraints, 0.0)
								// Run until accurate
								// .repeatedly()
								// End condition of the pathfind
								.until(() -> {
									Pose2d retPos = poseSupplier.get();
									return retPos.getTranslation()
											.getDistance(optLine.get()
													.optimizedLine()
													.end()) <= toleranceTranslationalMeters;
								})
								// Only re-range if outside bounds
								.onlyIf(() -> inRangeArea.get() == null || !inRangeArea.get()
										.contains(new Vector2(poseSupplier.get().getX(), poseSupplier.get().getY())))
								// Enforce minimum check period limit to avoid getting stuck, run periodically
								.alongWith(Commands.waitSeconds(kRerangeCheckPeriodSeconds)), Set.of()).repeatedly()
										// Only run if applicable
										.onlyIf(() -> reRangeWhenOut),
								// Run the rotation command to look at the target, rotate until within tolerance
								new DeferredCommand(
										() -> Commands.run(() -> rotateFunc.apply(lookAt.get(), poseSupplier.get()))
												.until(() -> endAfterRotating.getAsBoolean()
														&& angleWithinTolerance(lookAt.get(),
																poseSupplier.get().getRotation(), angleToleranceDegs)),
										// .finallyDo(PPHolonomicDriveController::clearRotationFeedbackOverride),
										Set.of())))
						.finallyDo(() -> {
							stopRotation.run();
							PPHolonomicDriveController.clearRotationFeedbackOverride();
						})
						.onlyIf(() -> optLine.get() != null && optLine.get().success());
	}

	/**
	 * Tries to auto-range using PathPlanner's {@link AutoBuilder} <b>once</b>, optionally until
	 * when the timeout is exceeded. <b> Just like other methods, this method is not
	 * alliance-aware, and doesn't flip the path automatically. Provide appropriately flipped
	 * coordinates.</b> AutoBuilder must be configured. For more accurate ranging, see
	 * {@link #autoRangeAndLookAt(Supplier, Supplier, PathConstraints, Double, double, double, Supplier, Supplier, BiFunction, Runnable, BooleanSupplier, boolean)}.
	 * This command also doesn't check if the target is in-range already. <b>Returns a deferred
	 * command.</b> Recommended to bind to a {@link Trigger} instance.
	 *
	 * @param optLine                      The optimized line trajectory. Obtained via the
	 *                                     optimization methods.
	 * @param lookAt                       The rotation to look at, set to robot's current angle to
	 *                                     discard.
	 * @param constraints                  The path constraints.
	 * @param endSpeedMps                  The end speed in m/s, set to 0.0 to stop at the end
	 *                                     pose.
	 * @param maxTime                      (Nullable) The maximum time to attempt for.
	 * @param toleranceTranslationalMeters The translational tolerance, in meters.
	 * @param angleToleranceDegs           The angular tolerance, in degrees.
	 * @return The command to pathfind, as a <b>deferred command</b>.
	 * @see #autoRangeAndLookAt_noDefer(OptimizedLine, Rotation2d, PathConstraints, double, Double,
	 *      double, double)
	 * @see #autoRangeAndLookAt(Supplier, Supplier, PathConstraints, Double, double, double,
	 *      Supplier, Supplier, BiFunction, Runnable, BooleanSupplier, boolean)
	 * @see #ofAllianceAwareLine(Translation2d, Translation2d)
	 */
	public static Command autoRangeAndLookAt(Supplier<OptimizedLine> optLine, Supplier<Rotation2d> lookAt,
			PathConstraints constraints, double endSpeedMps, Double maxTime, double toleranceTranslationalMeters,
			double angleToleranceDegs) {
		if (maxTime != null) return new DeferredCommand(() -> AutoBuilder
				// Build pathfind command to get to the desired pose
				.pathfindToPose(new Pose2d(optLine.get().optimizedLine().end(), lookAt.get()), constraints, endSpeedMps)
				// Apply a timeout to the command
				.withTimeout(maxTime.doubleValue()), Set.of()).onlyIf(() -> optLine.get().success());

		return new DeferredCommand(() -> AutoBuilder
				// Build pathfind command to get to the desired pose
				.pathfindToPose(new Pose2d(optLine.get().optimizedLine().end(), lookAt.get()), constraints,
						endSpeedMps),
				Set.of()).onlyIf(() -> optLine.get().success());
	}

	/**
	 * Repeatedly tries to auto-range using PathPlanner's {@link AutoBuilder}, until within the
	 * tolerance or optionally when the timeout is exceeded. <b> Just like other methods, this
	 * method is not alliance-aware, and doesn't flip the path automatically. Provide appropriately
	 * flipped coordinates.</b> AutoBuilder must be configured. More accurate compared to
	 * {@link #autoRangeAndLookAt_noDefer(OptimizedLine, Rotation2d, PathConstraints, double, Double, double, double)}.
	 * <b>This method does not return a deferred command.</b>
	 *
	 * <p>
	 * This method may take a really long time to run the first time, but will run faster. Still,
	 * this method is relatively <b>performance intensive</b>.
	 *
	 * @param optLine                      The optimized line trajectory. Obtained via the
	 *                                     optimization methods.
	 * @param lookAt                       The rotation to look at, set to robot's current angle to
	 *                                     discard.
	 * @param constraints                  The path constraints.
	 * @param maxTime                      (Nullable) The maximum time to attempt for. Highly
	 *                                     recommended to set a value.
	 * @param toleranceTranslationalMeters The translational tolerance, in meters.
	 * @param angleToleranceDegs           The angular tolerance, in degrees.
	 * @param inRangeArea                  If the robot is in this area, the request is ignored.
	 *                                     Put a null supplier {@code () -> null} to ignore.
	 * @param poseSupplier                 The supplier for the robot's estimated pose.
	 * @param rotateFunc                   The method to rotate the robot to the given rotation,
	 *                                     given the rotation setpoint and current pose. Applies,
	 *                                     then returns the calculated angular velocity. Units
	 *                                     should be rad/s.
	 * @param stopRotation                 The method to stop ONLY the robot's rotation.
	 * @param endAfterRotating             Whether or not to end the command after the rotational
	 *                                     setpoint is reached after pathfind, or to continue the
	 *                                     command until interruption.
	 * @return The command to pathfind, as a <b>non-deferred command</b>.
	 * @see #autoRangeAndLookAt(Supplier, Supplier, PathConstraints, Double, double, double,
	 *      Supplier, Supplier, BiFunction, Runnable, BooleanSupplier, boolean)
	 * @see #autoRangeAndLookAt_noDefer(OptimizedLine, Rotation2d, PathConstraints, double, Double,
	 *      double, double)
	 * @see #ofAllianceAwareLine(Translation2d, Translation2d)
	 */
	public static Command autoRangeAndLookAt_noDefer(OptimizedLine optLine, Rotation2d lookAt,
			PathConstraints constraints, Double maxTime, double toleranceTranslationalMeters, double angleToleranceDegs,
			Supplier<Convex> inRangeArea, Supplier<Pose2d> poseSupplier,
			BiFunction<Rotation2d, Pose2d, AngularVelocity> rotateFunc, Runnable stopRotation,
			boolean endAfterRotating) {
		if (maxTime != null) return Commands.runOnce(() -> AutoBuilder
				// Build pathfind command to get to the translational pose
				.pathfindToPose(new Pose2d(optLine.optimizedLine().end(), poseSupplier.get().getRotation()),
						constraints, 0.0)
				// Run until accurate
				.repeatedly()
				// End condition of the pathfind
				.until(() -> {
					Pose2d retPos = poseSupplier.get();
					return retPos.getTranslation()
							.getDistance(optLine.optimizedLine().end()) <= toleranceTranslationalMeters;
				})
				// No need to run if in range, just look at target
				.onlyIf(() -> inRangeArea.get() == null || !inRangeArea.get()
						.contains(new Vector2(poseSupplier.get().getX(), poseSupplier.get().getY()))))
				// Apply rotational override to the pathplanner path
				.alongWith(Commands.runOnce(() -> {
					PPHolonomicDriveController.overrideRotationFeedback(
							() -> rotateFunc.apply(lookAt, poseSupplier.get()).in(RadiansPerSecond));
				}))
				// Run the rotation command to look at the target, rotate until within tolerance
				.andThen(
						new DeferredCommand(
								() -> Commands.run(() -> rotateFunc.apply(lookAt, poseSupplier.get()))
										.until(() -> endAfterRotating && angleWithinTolerance(lookAt,
												poseSupplier.get().getRotation(), angleToleranceDegs))
										.finallyDo(PPHolonomicDriveController::clearRotationFeedbackOverride),
								Set.of()))
				// Apply a timeout to the whole command
				.withTimeout(maxTime.doubleValue())
				.finallyDo(stopRotation)
				.onlyIf(() -> optLine != null && optLine.success());

		return Commands.runOnce(() -> AutoBuilder
				// Build pathfind command to get to the translational pose
				.pathfindToPose(new Pose2d(optLine.optimizedLine().end(), poseSupplier.get().getRotation()),
						constraints, 0.0)
				// Run until accurate
				.repeatedly()
				// End condition of the pathfind
				.until(() -> {
					Pose2d retPos = poseSupplier.get();
					return retPos.getTranslation()
							.getDistance(optLine.optimizedLine().end()) <= toleranceTranslationalMeters;
				})
				// No need to run if in range, just look at target
				.onlyIf(() -> inRangeArea.get() == null || !inRangeArea.get()
						.contains(new Vector2(poseSupplier.get().getX(), poseSupplier.get().getY()))))
				// Apply rotational override to the pathplanner path
				.alongWith(Commands.runOnce(() -> {
					PPHolonomicDriveController.overrideRotationFeedback(
							() -> rotateFunc.apply(lookAt, poseSupplier.get()).in(RadiansPerSecond));
				}))
				// Run the rotation command to look at the target, rotate until within tolerance
				.andThen(
						new DeferredCommand(
								() -> Commands.run(() -> rotateFunc.apply(lookAt, poseSupplier.get()))
										.until(() -> endAfterRotating && angleWithinTolerance(lookAt,
												poseSupplier.get().getRotation(), angleToleranceDegs))
										.finallyDo(PPHolonomicDriveController::clearRotationFeedbackOverride),
								Set.of()))
				.finallyDo(stopRotation)
				.onlyIf(() -> optLine != null && optLine.success());

	}

	/**
	 * Tries to auto-range using PathPlanner's {@link AutoBuilder} <b>once</b>, optionally until
	 * when the timeout is exceeded. <b> Just like other methods, this method is not
	 * alliance-aware, and doesn't flip the path automatically. Provide appropriately flipped
	 * coordinates.</b> AutoBuilder must be configured. For more accurate ranging, see
	 * {@link #autoRangeAndLookAt_noDefer(OptimizedLine, Rotation2d, PathConstraints, Double, double, double, Supplier, Supplier, BiFunction, Runnable, boolean)}.
	 * This command also doesn't check if the target is in-range already. <b>This method does not
	 * return a deferred command.</b>
	 *
	 * @param optLine                      The optimized line trajectory. Obtained via the
	 *                                     optimization methods.
	 * @param lookAt                       The rotation to look at, set to robot's current angle to
	 *                                     discard.
	 * @param constraints                  The path constraints.
	 * @param endSpeedMps                  The end speed in m/s, set to 0.0 to stop at the end
	 *                                     pose.
	 * @param maxTime                      (Nullable) The maximum time to attempt for.
	 * @param toleranceTranslationalMeters The translational tolerance, in meters.
	 * @param angleToleranceDegs           The angular tolerance, in degrees.
	 * @return The command to pathfind, as a <b>non-deferred command</b>.
	 * @see #autoRangeAndLookAt(Supplier, Supplier, PathConstraints, double, Double, double,
	 *      double)
	 * @see #autoRangeAndLookAt_noDefer(OptimizedLine, Rotation2d, PathConstraints, Double, double,
	 *      double, Supplier, Supplier, BiFunction, Runnable, boolean)
	 * @see #ofAllianceAwareLine(Translation2d, Translation2d)
	 */
	public static Command autoRangeAndLookAt_noDefer(OptimizedLine optLine, Rotation2d lookAt,
			PathConstraints constraints, double endSpeedMps, Double maxTime, double toleranceTranslationalMeters,
			double angleToleranceDegs) {
		if (maxTime != null) return Commands.runOnce(() -> AutoBuilder
				// Build pathfind command to get to the desired pose
				.pathfindToPose(new Pose2d(optLine.optimizedLine().end(), lookAt), constraints, endSpeedMps)
				// Apply a timeout to the command
				.withTimeout(maxTime.doubleValue())).onlyIf(() -> optLine.success());

		return Commands.runOnce(() -> AutoBuilder
				// Build pathfind command to get to the translational pose
				.pathfindToPose(new Pose2d(optLine.optimizedLine().end(), lookAt), constraints, endSpeedMps))
				.onlyIf(() -> optLine.success());
	}

	/**
	 * Uses PathPlanner flipLineWithAlliance to flip the line with the alliance.
	 *
	 * @param lineToFlip The line to flip, blue-alliance relative.
	 * @return The flipped line, if appropriate.
	 */
	public static Line flipLineWithAlliance(Line lineToFlip) {
		Translation2d start = AllianceUtil.flipWithAlliance(new Pose2d(lineToFlip.start(), Rotation2d.kZero))
				.getTranslation();
		Translation2d end = AllianceUtil.flipWithAlliance(new Pose2d(lineToFlip.end(), Rotation2d.kZero))
				.getTranslation();

		return new Line(start, end);
	}

	/**
	 * Uses PathPlanner flipLineWithAlliance to flip the line with the alliance.
	 *
	 * @param lineToFlip The optimized line to flip, relative to the blue alliance.
	 * @return The flipped line, if appropriate.
	 */
	public static OptimizedLine flipOptimizedLineWithAlliance(OptimizedLine lineToFlip) {
		Translation2d start = AllianceUtil
				.flipWithAlliance(new Pose2d(lineToFlip.optimizedLine().start(), Rotation2d.kZero))
				.getTranslation();
		Translation2d end = AllianceUtil
				.flipWithAlliance(new Pose2d(lineToFlip.optimizedLine().end(), Rotation2d.kZero))
				.getTranslation();

		return new OptimizedLine(new Line(start, end), lineToFlip.success());
	}

	/**
	 * Creates an alliance-aware line segment, flipped for red alliance.
	 *
	 * @param blueRelativeStart The start point of the line, relative to the blue alliance.
	 * @param blueRelativeEnd   The end point of the line, relative to the blue alliance.
	 * @return An alliance-aware line segment, flipped for red alliance.
	 */
	public static Line ofAllianceAwareLine(Translation2d blueRelativeStart, Translation2d blueRelativeEnd) {
		return flipLineWithAlliance(new Line(blueRelativeStart, blueRelativeEnd));
	}

	/**
	 * Creates a supplier which returns a Rotation2d which directly faces the given target from the
	 * robot's pose.
	 *
	 * @param robotPose  The supplier for the robot's pose.
	 * @param targetPose The supplier for the target's pose.
	 * @return The supplier for the Rotation2d.
	 */
	public static Supplier<Optional<Rotation2d>> createLookAtAngleSupplier(Supplier<Pose2d> robotPose,
			Supplier<Pose2d> targetPose) {
		// Aim at target
		return () -> {
			Pose2d bot = robotPose.get();
			Pose2d target = targetPose.get();
			double dy = target.getY() - bot.getY();
			double dx = target.getX() - bot.getX();

			// Fallback to current heading if target is on top of robot
			if (Math.hypot(dy, dx) < kLookAtMinDist) return Optional.empty();

			// For a right angle triangle, tanA = dy / dx
			// A = atan(dy / dx), we use atan2 for the -pi to pi range
			return Optional.of(new Rotation2d(Math.atan2(dy, dx)));
		};
	}

	/**
	 * Returns whether the given current Rotation2d is near the other desired Rotation2d by the
	 * given amount.
	 *
	 * @param desired The desired rotation from 0 to 360 degrees.
	 * @param current The current rotation from 0 to 360 degrees.
	 * @param tolDeg  The angle tolerance, in degrees.
	 * @return Whether the current angle is near the other angle.
	 */
	public static boolean angleWithinTolerance(Rotation2d desired, Rotation2d current, double tolDeg) {
		// Uses the shortest angular difference
		double diffDeg = Math.abs(desired.minus(current).getDegrees());
		if (diffDeg > 180.0) diffDeg = 360.0 - diffDeg;
		return diffDeg <= tolDeg;
	}

	/**
	 * A record instance representing a line segment.
	 *
	 * @param start The starting point of the line segment.
	 * @param end   The ending point of the line segment.
	 */
	public static record Line(Translation2d start, Translation2d end) {}

	/**
	 * A record instance representing an optimized line segment.
	 *
	 * @param optimizedLine The optimized line segment.
	 * @param success       Whether the optimization was successful.
	 */
	public static record OptimizedLine(Line optimizedLine, boolean success) {}

	/** Returns whether the arrangement is in counter-clockwise order. */
	private static boolean isCCW(Translation2d A, Translation2d B, Translation2d C) {
		return (C.getY() - A.getY()) * (B.getX() - A.getX()) > (B.getY() - A.getY()) * (C.getX() - A.getX());
	}

	/** Returns whether the robot collides with the obstacle. */
	private static boolean rectCollision(Line shot, Line avoid, double halfWidth, double halfLength) {
		Translation2d A = shot.start;
		Translation2d B = shot.end;
		double L = A.getDistance(B);
		if (L < 1e-6) return false;

		// Build unit-dir && perp
		double dx = (B.getX() - A.getX()) / L;
		double dy = (B.getY() - A.getY()) / L;
		double pdx = -dy, pdy = dx;

		// Project world->frame: x along shot, y perp
		BiFunction<Translation2d, double[], double[]> proj = (P, buf) -> {
			double vx = P.getX() - A.getX();
			double vy = P.getY() - A.getY();
			buf[0] = vx * dx + vy * dy;
			buf[1] = vx * pdx + vy * pdy;
			return buf;
		};

		double[] p1buf = new double[2];
		double[] p2buf = new double[2];
		Translation2d P1 = new Translation2d(proj.apply(avoid.start, p1buf)[0], proj.apply(avoid.start, p1buf)[1]);
		Translation2d P2 = new Translation2d(proj.apply(avoid.end, p2buf)[0], proj.apply(avoid.end, p2buf)[1]);
		Line obs = new Line(P1, P2);

		// Rectangle bounds in frame
		double minX = -halfLength;
		double maxX = L + halfLength;
		double minY = -halfWidth;
		double maxY = halfWidth;

		// Endpoint-inside
		if ((P1.getX() >= minX && P1.getX() <= maxX && P1.getY() >= minY && P1.getY() <= maxY)
				|| (P2.getX() >= minX && P2.getX() <= maxX && P2.getY() >= minY && P2.getY() <= maxY)) {
			return true;
		}

		// Edge intersections
		// Build four frame-space edges
		Line e1 = new Line(new Translation2d(minX, minY), new Translation2d(maxX, minY));
		Line e2 = new Line(new Translation2d(maxX, minY), new Translation2d(maxX, maxY));
		Line e3 = new Line(new Translation2d(maxX, maxY), new Translation2d(minX, maxY));
		Line e4 = new Line(new Translation2d(minX, maxY), new Translation2d(minX, minY));

		return doesIntersectOnce(obs, e1) || doesIntersectOnce(obs, e2) || doesIntersectOnce(obs, e3)
				|| doesIntersectOnce(obs, e4);
	}
}
