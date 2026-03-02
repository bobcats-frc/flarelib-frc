package com.bobcats.lib.auto;

import com.bobcats.lib.utils.AllianceUtil;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;
import java.util.stream.Collectors;

/**
 * A utility class used to generate, manage and select autonomous routines on the fly.
 *
 * <p>
 * Call {@link AutonomousManager#updateSelectedCommand()} to update the retrieved command
 * manually, as it's only updated automatically when the autonomous period begins.
 *
 * <p>
 * <i><b>Note: Only 1 instance of the autonomous manager is allowed!</b></i>
 */
public class AutonomousManager {
	// Maps && Constraints
	private final Map<String, Pose2d> m_poseMap;
	private final Map<String, Command> m_commandMap;
	private Map<String, Trigger> m_zoneTriggers;
	private PathConstraints m_constraints;

	// Autonomous Suppliers
	private SendableChooser<Command> m_commandChooser;
	private Supplier<Command> m_autoSupplier;
	private Supplier<Pose2d> m_robotPoseSupplier;

	// Strategies
	private PrimaryCommandSource m_primaryCommandSource = PrimaryCommandSource.kNone;
	private PathfindStrategy m_pathfindStrategy = PathfindStrategy.kDefault;

	// Commands
	private Command m_mainRetrievedCommand = Commands.none();
	private Command m_fallbackCommand = Commands.none();

	// Pre-auto Command
	private Command m_preAutoCommand = Commands.none();
	private boolean m_isParallelPre = true;
	private boolean m_isPreCommandSet = false;

	// Dashboard Data
	private String m_dashboardPatternInputKey = "Autonomous Pattern";

	// Singleton instance
	private static AutonomousManager m_instance;

	// Config Variables
	/**
	 * Whether to throw an error or to warn via DS. Generally should only be used when testing and
	 * not in competition. Defaults to false.
	 */
	public static boolean kErrorInsteadOfWarn = false;

	/**
	 * The autonomous key to use with NetworkTables. Defaults to "Auto".
	 */
	public static String kNTAutonomousKey = "Auto";

	private static final String kPatternCommandName = "Pattern Generated Routine";
	private static final String kCommandChooserDashboardKey = "Autonomous Chooser";
	private static final String kDashboardCmdNamePreviewKey = "Selected Autonomous Command (Preview)";

	/**
	 * Constructs a new AutonomousManager.
	 *
	 * @param poses       The map of pose names to poses.
	 * @param commands    The map of command names to commands.
	 * @param constraints The path constraints to use for pathfinding.
	 */
	public AutonomousManager(Map<String, Pose2d> poses, Map<String, Command> commands, PathConstraints constraints) {
		this(poses, commands, constraints, true);
	}

	/**
	 * Constructs a new AutonomousManager.
	 *
	 * @param poses               The map of pose names to poses.
	 * @param commands            The map of command names to commands.
	 * @param constraints         The path constraints to use for pathfinding.
	 * @param autoScheduleCommand Whether to automatically schedule the command when the autonomous
	 *                            period begins.
	 */
	public AutonomousManager(Map<String, Pose2d> poses, Map<String, Command> commands, PathConstraints constraints,
			boolean autoScheduleCommand) {
		this(poses, commands, constraints, autoScheduleCommand, false);
	}

	/**
	 * Constructs a new AutonomousManager.
	 *
	 * <p>
	 * Note that {@link AutoBuilder} needs to be configured if warmupPathfind is set to true.
	 *
	 * @param poses               The map of pose names to poses.
	 * @param commands            The map of command names to commands.
	 * @param constraints         The path constraints to use for pathfinding.
	 * @param autoScheduleCommand Whether to automatically schedule the command when the autonomous
	 *                            period begins.
	 * @param warmupPathfind      Whether to call the PathPlanner pathfind warmup command.
	 *
	 * @throws RuntimeException If warmupPathfind is true while AutoBuilder is not configured, and
	 *                          {@link #kErrorInsteadOfWarn} is set to true.
	 */
	public AutonomousManager(Map<String, Pose2d> poses, Map<String, Command> commands, PathConstraints constraints,
			boolean autoScheduleCommand, boolean warmupPathfind) {
		if (m_instance != null)
			throw new IllegalStateException("AutonomousManager already initialized, only one instance is allowed");
		if (poses == null) throw new IllegalArgumentException("Poses cannot be null");
		if (commands == null) throw new IllegalArgumentException("Commands cannot be null");
		if (constraints == null) throw new IllegalArgumentException("Constraints cannot be null");

		m_poseMap = poses;
		m_commandMap = commands;
		m_constraints = constraints;

		if (autoScheduleCommand) {
			RobotModeTriggers.autonomous()
					.onTrue(Commands.runOnce(this::scheduleAuto))
					.onFalse(Commands.runOnce(this::cancelAuto));
		}

		if (warmupPathfind && AutoBuilder.isConfigured())
			CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
		else if (warmupPathfind && !AutoBuilder.isConfigured())
			warnOrThrow("AutoBuilder not configured for warmupPathfind");

		updateNTEntries();
		// Intialize the dashboard key
		// SmartDashboard.getString(m_dashboardPatternInputKey, "");
		m_instance = this;
	}

	/**
	 * Generates an autonomous routine from the given string pattern. Uses the poses and commands
	 * provided in the constructor. Keep in mind that the {@link AutoBuilder} needs to be
	 * configured if any poses are used. Command names are case-sensitive. If duplicate
	 * command-pose names are used, the command will be prioritized.
	 *
	 * <p>
	 * <b>Note: The command names used in the string can't contain "/", ":" or spaces.</b>
	 *
	 * <p>
	 * Example usage:
	 *
	 * <pre>
	 * <code>
	 * String str = "path1/path2:command1:command2/path3:command3";
	 * m_generator.generateAutoFromString(str); // Would generate: PATH1 -> (PATH2 COMMAND1 COMMAND2 parallel) -> (PATH3
	 *                                          // COMMAND3 parallel)
	 * </code> </pre>
	 *
	 *
	 * @param pathPattern The path pattern. The commands and path names separated by a '/'. To make
	 *                    2 or more parts parallel, use a ':' inbetween each.
	 * @return The generated autonomous command, or an empty command if the pattern is invalid or
	 *         empty.
	 * @throws RuntimeException If the path pattern is malformed and {@link #kErrorInsteadOfWarn}
	 *                          is true.
	 */
	public Command generateAutoFromString(String pathPattern) {
		if (pathPattern == null || pathPattern.isBlank()) {
			warnOrThrow("null or empty path pattern passed");
			return Commands.none();
		}

		ArrayList<Command> sequenced = new ArrayList<>();

		// Go through each "/" section
		for (String sequentialSection : pathPattern.split("/")) {
			String[] parallelSections = sequentialSection.split(":");
			if (sequentialSection.isBlank() || parallelSections.length == 0) {
				warnOrThrow("empty sequenced section passed");
				continue;
			}

			List<Command> parallel = Arrays.stream(parallelSections)
					.map(String::trim) // Remove whitespace
					.filter(s -> {
						// Filter empty parallel sections and warn
						boolean isBlank = s.isBlank();
						if (isBlank) warnOrThrow("empty parallel command name passed");
						return !isBlank;
					})
					.map(name -> {
						Command cmd = m_commandMap.get(name);
						if (cmd != null) { return cmd; }

						Pose2d pose = AllianceUtil.flipWithAlliance(m_poseMap.get(name));
						if (pose != null) return getPathfindCommandPrivate(pose, name, 0);

						warnOrThrow("invalid command or path name '" + name + "'");
						return null;
					})
					.filter(Objects::nonNull)
					.collect(Collectors.toList());

			// Add to sequence if not empty
			if (!parallel.isEmpty()) {
				sequenced.add(parallel.size() == 1 // A small optimization to not add a singular command into a
						// parallel command group
						? parallel.get(0) : Commands.parallel(parallel.toArray(Command[]::new)));
			}
		}

		Command finalCommand = Commands.sequence(sequenced.toArray(Command[]::new));
		finalCommand.setName(kPatternCommandName + " (" + pathPattern + ")");
		return finalCommand;
	}

	// Routine Suppliers //

	/**
	 * Initializes the command autonomous sendable chooser.
	 *
	 * @return The singleton instance for chaining.
	 */
	public AutonomousManager initCommandAutoChooser() {
		if (m_commandChooser != null) {
			warnOrThrow("command sendable chooser already initialized, skipping");
			return m_instance;
		}

		m_commandChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData(kCommandChooserDashboardKey, m_commandChooser);
		return m_instance;
	}

	/**
	 * Initializes the command autonomous sendable chooser with the given default option.
	 *
	 * @param defaultAuto The default routine to select.
	 * @return The singleton instance for chaining.
	 */
	public AutonomousManager initCommandAutoChooser(String defaultAuto) {
		if (m_commandChooser != null) {
			warnOrThrow("command sendable chooser already initialized, skipping");
			return m_instance;
		}

		m_commandChooser = AutoBuilder.buildAutoChooser(defaultAuto);
		SmartDashboard.putData(kCommandChooserDashboardKey, m_commandChooser);
		return m_instance;
	}

	/**
	 * Returns the autonomous command chooser.
	 *
	 * @return The chooser.
	 */
	public SendableChooser<Command> getCommandAutoChooser() { return m_commandChooser; }

	/**
	 * Sets the autonomous command supplier. Only used when the primary source is
	 * {@link PrimaryCommandSource#kSupplier}.
	 *
	 * @param supplier The supplier.
	 * @return The singleton instance for chaining.
	 */
	public AutonomousManager setAutoSupplier(Supplier<Command> supplier) {
		m_autoSupplier = supplier;
		return m_instance;
	}

	/**
	 * Sets which source to get the autonomous command from.
	 *
	 * @param option The option to set.
	 * @return The singleton instance for chaining.
	 */
	public AutonomousManager setPrimaryCommandSource(PrimaryCommandSource option) {
		m_primaryCommandSource = option;
		return m_instance;
	}

	/**
	 * Makes the autonomous manager use a custom routine builder to get the autonomous command.
	 *
	 * @param builder The routine builder.
	 */
	public void setCustomRoutineBuilder(CustomRoutineBuilder builder) {
		setAutoSupplier(builder::getRoutine);
		setPrimaryCommandSource(PrimaryCommandSource.kSupplier);
	}

	/**
	 * Returns the autonomous command based on the primary source. <b>May return null if a faulty
	 * supplier is passed or an error is thrown. Does not account for the fallback command.</b>
	 *
	 * @return The autonomous command.
	 * @throws RuntimeException If an error occurs while getting the command and if
	 *                          {@link #kErrorInsteadOfWarn} is set to true.
	 */
	public Command getAutonomousCommand() {
		try {
			Command cmd;
			switch (m_primaryCommandSource) {
				case kNone: {
					cmd = null;
					break;
				}

				case kSupplier: {
					if (m_autoSupplier == null) {
						warnOrThrow("no supplier was provided");
						cmd = null;
					} else {
						cmd = m_autoSupplier.get();
					}
					break;
				}

				case kChooserCommand: {
					if (m_commandChooser == null) {
						warnOrThrow("command sendable chooser not initialized");
						cmd = null;
					} else {
						cmd = m_commandChooser.getSelected();
					}
					break;
				}

				case kDashboardPatternStringInput: {
					cmd = generateAutoFromString(SmartDashboard.getString(m_dashboardPatternInputKey, ""));
					break;
				}

				default: {
					warnOrThrow("invalid command source");
					cmd = null;
					break;
				}
			}

			// Apply the pre-auto command
			if (cmd != null && m_isPreCommandSet && m_preAutoCommand != null) {
				String name = cmd.getName();
				cmd = m_isParallelPre ? Commands.parallel(m_preAutoCommand, cmd)
						: Commands.sequence(m_preAutoCommand, cmd);
				cmd.setName(name);
			}
			return cmd;
		} catch (Exception e) {
			warnOrThrow(
					"error while getting autonomous command (src=" + m_primaryCommandSource + "): " + e.getMessage());
			return Commands.none();
		}
	}

	/**
	 * Returns the retrieved auto command. Defaults to <code>Commands.none()</code>, and is <b>only
	 * updated when the autonomous period begins, or when {@link #updateSelectedCommand()} is
	 * called.</b>
	 *
	 * @return The command.
	 */
	public Command getRetrievedCommand() { return m_mainRetrievedCommand; }

	// Dashboard String Input //

	/**
	 * Sets the key to use for the dashboard input. This is only used when the primary input mode
	 * is set to {@link PrimaryCommandSource#kDashboardPatternStringInput}.
	 *
	 * <p>
	 * Defauts to "Autonomous Pattern".
	 *
	 * @param key The dashboard key.
	 * @return The singleton instance for chaining.
	 */
	public AutonomousManager setDashboardPatternInputKey(String key) {
		m_dashboardPatternInputKey = key;
		// Intialize the dashboard key if it doesn't exist.
		SmartDashboard.getString(m_dashboardPatternInputKey, "");
		return m_instance;
	}

	/**
	 * Returns the key used for the dashboard pattern input.
	 *
	 * @return The dashboard key.
	 */
	public String getDashboardPatternInputKey() { return m_dashboardPatternInputKey; }

	// Force-scheduling //

	// /**
	// * Forcefully schedules the autonomous routine.
	// */
	// public void forceStartAutonomousCommand() {
	// System.out.println("Force-starting the autonomous command...");
	// scheduleAuto();
	// }

	// /**
	// * Forcefully ends the autonomous routine.
	// */
	// public void forceEndAutonomousCommand() {
	// System.out.println("Force-ending the autonomous routine...");
	// cancelAuto();
	// }

	// Command Updates //

	/**
	 * Updates the selected command preview on the dashboard.
	 */
	public void updateSelectedCommandPreview() {
		Command cmd = getAutonomousCommand();
		String name = (cmd != null) ? cmd.getName() : "N/A";
		SmartDashboard.putString(kDashboardCmdNamePreviewKey, name);
	}

	/**
	 * Updates the retrieved autonomous command.
	 */
	public void updateSelectedCommand() {
		m_mainRetrievedCommand = getAutonomousCommand();
	}

	// Related Commands //

	/**
	 * Sets the fallback autonomous command for when the main autonomous command fails. This
	 * command will only be used if the <code>autoScheduleCommand</code> parameter is set to true,
	 * unless called manually.
	 *
	 * @param fallback The fallback command.
	 * @return The singleton instance for chaining.
	 */
	public AutonomousManager setFallbackCommand(Command fallback) {
		m_fallbackCommand = fallback;
		return m_instance;
	}

	/**
	 * Returns the current fallback command.
	 *
	 * @return The fallback command.
	 */
	public Command getFallbackCommand() { return m_fallbackCommand; }

	/**
	 * Sets the command to schedule right before starting the autonomous routine.
	 *
	 * @param preCommand The commannd to attach.
	 * @param isParallel Whether to run the command in parallel with the autonomous routine.
	 * @return The singleton instance for chaining.
	 */
	public AutonomousManager setPreAutoCommand(Command preCommand, boolean isParallel) {
		m_preAutoCommand = preCommand;
		m_isParallelPre = isParallel;
		m_isPreCommandSet = true;
		return m_instance;
	}

	// Pathfinding //
	/**
	 * Sets the pathfinding strategy to use for the autonomous manager. Defaults to
	 * {@link PathfindStrategy#kDefault}.
	 *
	 * @param strategy The new pathfinding strategy to use.
	 * @return The singleton instance for chaining.
	 */
	public AutonomousManager setPathfindStrategy(PathfindStrategy strategy) {
		m_pathfindStrategy = strategy;
		return m_instance;
	}

	/**
	 * Returns the pathfinding strategy used for the autonomous manager.
	 *
	 * @return The strategy.
	 */
	public PathfindStrategy getPathfindStrategy() { return m_pathfindStrategy; }

	/**
	 * Pathfinds to the given pose according to the pathfind strategy.
	 *
	 * @param poseName              The name of the pose to pathfind to, must be in the pose map.
	 * @param pathfindNegligibility The tolerance below which the pathfind to the starting point of
	 *                              the pose is not applied if using a precise pathfind.
	 * @return The command, empty if an invalid pose is provided.
	 * @throws RuntimeException If an error occurs while retrieving the command and if
	 *                          {@link #kErrorInsteadOfWarn} is set to true.
	 */
	public Command getPathfindCommand(String poseName, double pathfindNegligibility) {
		Pose2d pose = m_poseMap.get(poseName);
		return getPathfindCommandPrivate(pose, poseName, pathfindNegligibility);
	}

	/**
	 * Sets the pose triggers. Requires the pose supplier to be set via
	 * {@link #setRobotPoseSupplier(Supplier)} first.
	 *
	 * @param triggers The list of zone triggers.
	 * @return The singleton instance for chaining.
	 */
	public AutonomousManager initializePoseTriggers(List<ZoneTrigger> triggers) {
		if (m_zoneTriggers != null) { warnOrThrow("pose triggers already initialized, skipping"); return m_instance; }
		if (triggers == null || triggers.isEmpty()) {
			warnOrThrow("null or empty triggers list passed");
			return m_instance;
		}

		m_zoneTriggers = new HashMap<>();

		// Convert zone triggers into a bool trigger
		for (ZoneTrigger trigger : triggers) {
			if (trigger == null || trigger.getPoseName() == null || trigger.getCommand() == null) {
				warnOrThrow("null trigger or invalid trigger parameters passed");
				continue;
			}

			// Create a new trigger for the zone
			Trigger zoneTrigger = new Trigger(() -> {
				Pose2d robotPose = m_robotPoseSupplier != null ? m_robotPoseSupplier.get() : null;
				Pose2d targetPose = AllianceUtil.flipWithAlliance(m_poseMap.get(trigger.getPoseName()));
				if (targetPose == null) {
					warnOrThrow("invalid pose name '" + trigger.getPoseName() + "' passed to trigger");
					return false;
				}
				if (robotPose == null) { warnOrThrow("null robot pose passed to trigger"); return false; }
				return robotPose != null && targetPose != null && robotPose.getTranslation()
						.getDistance(targetPose.getTranslation()) <= trigger.getTriggerDistance();
			});

			// Bind the command to the trigger
			zoneTrigger.whileTrue(trigger.getCommand());
			m_zoneTriggers.put(trigger.getPoseName(), zoneTrigger);
		}

		return m_instance;
	}

	/**
	 * Returns the zone trigger for the given pose name. The pose name must be registered in the
	 * pose map, and the triggers must be initialized via {@link #initializePoseTriggers(List)}.
	 *
	 * @param poseName The name of the pose to get the trigger for.
	 * @return The trigger, or null if not found.
	 */
	public Trigger getZoneTrigger(String poseName) {
		if (m_zoneTriggers == null) {
			warnOrThrow("pose triggers not initialized, call AutonomousManager::initializePoseTriggers first");
			return null;
		}
		if (poseName == null || poseName.isBlank()) { warnOrThrow("null or empty pose name passed"); return null; }

		Trigger trigger = m_zoneTriggers.get(poseName);
		if (trigger == null) { warnOrThrow("no trigger found for pose '" + poseName + "'"); }
		return trigger;
	}

	/**
	 * Sets the field-relative robot pose supplier.
	 *
	 * @param poseSupplier The pose supplier.
	 * @return The singleton instance for chaining.
	 */
	public AutonomousManager setRobotPoseSupplier(Supplier<Pose2d> poseSupplier) {
		if (poseSupplier == null) { warnOrThrow("null pose supplier passed"); return m_instance; }

		m_robotPoseSupplier = poseSupplier;
		return m_instance;
	}

	/**
	 * Returns the singleton {@link AutonomousManager} instance.
	 *
	 * @return The instance.
	 */
	public static AutonomousManager getInstance() { return m_instance; }

	/** Schedules the autonomous routine. */
	public void scheduleAuto() {
		m_mainRetrievedCommand = getAutonomousCommand();
		if (m_mainRetrievedCommand != null) {
			CommandScheduler.getInstance().schedule(m_mainRetrievedCommand);
			System.out.println("Starting autonomous routine: " + m_mainRetrievedCommand.getName());
		} else {
			DriverStation.reportWarning("No autonomous command found, attempting to use fallback", false);
			if (m_fallbackCommand != null) {
				CommandScheduler.getInstance().schedule(m_fallbackCommand);
				DriverStation.reportWarning("Starting fallback autonomous routine: " + m_fallbackCommand.getName(),
						false);
			} else {
				warnOrThrow("no fallback command found, using none");
			}
		}
	}

	/** Cancels the autonomous routine. */
	public void cancelAuto() {
		if (m_mainRetrievedCommand != null && m_mainRetrievedCommand.isScheduled()) m_mainRetrievedCommand.cancel();
		if (m_fallbackCommand != null && m_fallbackCommand.isScheduled()) m_fallbackCommand.cancel();
	}

	// Private Methods //

	private void warnOrThrow(String message) {
		if (kErrorInsteadOfWarn) throw new RuntimeException("AutonomousManager: " + message);
		else DriverStation.reportWarning("Error in AutonomousManager: " + message, true);
	}

	private void updateNTEntries() {
		NetworkTableInstance.getDefault()
				.getStringArrayTopic("/" + kNTAutonomousKey + "/Commands")
				.publish()
				.set(m_commandMap.keySet().toArray(String[]::new));
		NetworkTableInstance.getDefault()
				.getStringArrayTopic("/" + kNTAutonomousKey + "/Poses")
				.publish()
				.set(m_poseMap.keySet().toArray(String[]::new));
	}

	private Command getPathfindCommandPrivate(Pose2d pose, String poseName, double tol) {
		if (pose == null && m_pathfindStrategy != PathfindStrategy.kPrecisionPathfind) return Commands.none();
		pose = AllianceUtil.flipWithAlliance(pose);

		// The setup for pathfinding
		if (m_pathfindStrategy == PathfindStrategy.kPrecisionPathfind) {
			try {
				PathPlannerPath path = PathPlannerPath.fromPathFile(poseName);
				return Commands.either(AutoBuilder.followPath(path),
						AutoBuilder.pathfindThenFollowPath(path, m_constraints),
						() -> m_robotPoseSupplier != null && m_robotPoseSupplier.get()
								.getTranslation()
								.getDistance(
										path.getStartingHolonomicPose().orElse(Pose2d.kZero).getTranslation()) <= tol);
			} catch (Exception e) {
				warnOrThrow("failed to load path '" + poseName
						+ "' for precision pathfind command, using default pathfind, error: " + e.getMessage());
				return AutoBuilder.pathfindToPose(pose, m_constraints, 0.0);
			}
		} else {
			return AutoBuilder.pathfindToPose(pose, m_constraints, 0.0);
		}
	}

	// Enums && Classes //

	/**
	 * The primary source to get the autonomous command from for the {@link AutonomousManager}.
	 */
	public enum PrimaryCommandSource {
		/**
		 * Disables the autonomous mode. <i>The pre-auto command will still be ran.</i>
		 */
		kNone,
		/** Gets the command from the supplier provided. */
		kSupplier,
		/** Gets the command from the command sendable chooser. */
		kChooserCommand,
		/** Gets the command from a dashboard string input. */
		kDashboardPatternStringInput,
	}

	/**
	 * The primary pathfinding strategy for the {@link AutonomousManager}.
	 */
	public enum PathfindStrategy {
		/**
		 * Uses the built in {@link AutoBuilder#pathfindToPose(Pose2d, PathConstraints)} method to
		 * pathfind. <b>{@link AutoBuilder} must be configured.</b>
		 */
		kDefault,
		/**
		 * First pathfinds to the start of a path, and then follows the path which goes to a key
		 * location for increased precision. The paths should start at a point relatively close to the
		 * goal/ending point, and must have the same name as registered in the poses map.
		 * <b>{@link AutoBuilder} must be configured.</b>
		 */
		kPrecisionPathfind;
	}

	/**
	 * Creates a trigger zone to execute a command when the robot is close enough to a pose by the
	 * given amount.
	 */
	public static class ZoneTrigger {
		private final String m_poseName;
		private final double m_trigDist;
		private final Command m_command;

		/**
		 * Constructs a new ZoneTrigger.
		 *
		 * @param poseName    The name of origin of the trigger.
		 * @param triggerDist The distance from the pose to trigger the command.
		 * @param command     The command to execute when the robot is within the trigger distance.
		 */
		public ZoneTrigger(String poseName, double triggerDist, Command command) {
			if (poseName == null || poseName.isBlank() || command == null)
				throw new IllegalArgumentException("emtpy or null parameters passed");

			m_poseName = poseName;
			m_trigDist = triggerDist;
			m_command = command;
		}

		/**
		 * Returns the origin pose of the trigger.
		 *
		 * @return The pose name.
		 */
		public String getPoseName() { return m_poseName; }

		/**
		 * Returns the distance from the pose to trigger the command.
		 *
		 * @return The trigger distance.
		 */
		public double getTriggerDistance() { return m_trigDist; }

		/**
		 * Returns the command to execute.
		 *
		 * @return The command.
		 */
		public Command getCommand() { return m_command; }
	}
}
