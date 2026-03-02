package com.bobcats.lib.control.shooter;

import static com.bobcats.lib.utils.Utils.EPSILON;

import com.bobcats.lib.container.Vector3;
import com.bobcats.lib.control.shooter.data.ProjectileTarget;
import com.bobcats.lib.control.shooter.data.ShooterDescriptor;
import com.bobcats.lib.control.shooter.data.ShooterProjectile;
import com.bobcats.lib.control.shooter.data.ShooterProjectileType;
import com.bobcats.lib.math.PhysicsUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

// Modified from 6328.

/**
 * A shooter calculator allowing for on-the-fly shot calculations.
 */
public class ShooterCalculator {
	/**
	 * The maximum noise in the roll (X axis rotation) for projectile visualization, in radians. Do
	 * not increase too much due to it causing choppy rotation. Uses radians as the unit of angle.
	 */
	public double MaxRollNoiseRad = Math.toRadians(2);

	/**
	 * The list of projectile targets to use for trajectory visualization.
	 */
	public List<ProjectileTarget> ProjectileTargets = new ArrayList<>();

	/**
	 * The maximum time-of-flight to simulate for a given object, in seconds.
	 */
	public double TrajectoryMaxTime = 6;

	/**
	 * The amount of look-ahead steps to use for iterative convergence when calculating shots.
	 */
	public int IterativeConvergenceLookAheadSteps = 4;

	/**
	 * The record instance for the calculated shot parameters.
	 */
	public record ShooterParameters(boolean isValid, Rotation2d turretAngleField, Rotation2d turretAngleRobot,
			double hoodAngleDegs, double rollerSpeedsRPM, double timeOfFlight, Translation3d exitPose,
			Vector3 exitVelocityVec3, double exitVelocityMetersPerSec) {}

	private ShooterDescriptor m_descriptor;

	/**
	 * Constructs a new ShooterCalculator.
	 *
	 * @param descriptor The shooter descriptor containing all the necessary parameters.
	 */
	public ShooterCalculator(ShooterDescriptor descriptor) {
		m_descriptor = descriptor;
	}

	/**
	 * Calculates and returns the required parameters to hit a given target. Note that this is
	 * expected to be called periodically.
	 *
	 * @param target3d        The target's 3D location.
	 * @param robotPosition3d The robot's field-relative pose.
	 * @param robotAngle      The robot's current rotation.
	 * @param robotVelocity   The robot's field-relative chassis speeds.
	 * @param projectileType  The type of projectile being fired.
	 * @return The calculated shooter parameters.
	 */
	public ShooterParameters updateParameters(Translation3d target3d, Pose3d robotPosition3d, Rotation2d robotAngle,
			ChassisSpeeds robotVelocity, ShooterProjectileType projectileType) {
		if (m_descriptor.getAngleInterpolator() == null || m_descriptor.getRPMInterpolator() == null
				|| m_descriptor.getFlightTimeInterpolator() == null) {
			DriverStation.reportWarning(
					"WARNING: ShooterCalculator::updateParameters, attempted to calculate without valid interpolators",
					true);
			return new ShooterParameters(false, new Rotation2d(), new Rotation2d(), 0.0, 0.0, 0.0, new Translation3d(),
					new Vector3(0.0, 0.0, 0.0), 0.0);
		}

		double dz = target3d.getZ() - m_descriptor.getRobotToTurret().getZ();
		Transform2d robotToTurret = new Transform2d(m_descriptor.getRobotToTurret().getX(),
				m_descriptor.getRobotToTurret().getY(), m_descriptor.getRobotToTurret().getRotation().toRotation2d());
		// Predict robot position after shot delay, assuming constant velocity for very
		// short shotDelay
		Pose2d robotPosition = robotPosition3d.toPose2d()
				.exp(new Twist2d(robotVelocity.vxMetersPerSecond * m_descriptor.getShotDelay(),
						robotVelocity.vyMetersPerSecond * m_descriptor.getShotDelay(),
						robotVelocity.omegaRadiansPerSecond * m_descriptor.getShotDelay()));
		Pose2d robotPose2d = robotPosition3d.toPose2d();
		Translation2d target = target3d.toTranslation2d();
		// Calculate distance from turret's pivot to the target
		double distToTarget = target.getDistance(robotPose2d.getTranslation());
		// Calculate field-relative turret velocity
		// Obtained via v = w x r
		double vt_x = robotVelocity.vxMetersPerSecond
				+ robotVelocity.omegaRadiansPerSecond * (robotToTurret.getY() * Math.cos(robotAngle.getRadians())
						- robotToTurret.getX() * Math.sin(robotAngle.getRadians()));
		double vt_y = robotVelocity.vyMetersPerSecond
				+ robotVelocity.omegaRadiansPerSecond * (robotToTurret.getX() * Math.cos(robotAngle.getRadians())
						- robotToTurret.getY() * Math.sin(robotAngle.getRadians()));

		double timeOfFlight = 0;
		Pose2d offsetedPose = robotPose2d;
		double offsetedTargetDist = distToTarget;

		// Use iterative convergence
		for (int i = 0; i < IterativeConvergenceLookAheadSteps; i++) {
			timeOfFlight = m_descriptor.getFlightTimeInterpolator().interpolate(offsetedTargetDist, dz);
			// dx = dv*t
			double offsetX = vt_x * timeOfFlight;
			double offsetY = vt_y * timeOfFlight;
			offsetedPose = new Pose2d(robotPose2d.getTranslation().plus(new Translation2d(offsetX, offsetY)),
					robotPose2d.getRotation());
			offsetedTargetDist = target.getDistance(offsetedPose.getTranslation());
		}

		// Calculate some parameters accounted for the transferred velocity
		Rotation2d turretAngleField = target.minus(offsetedPose.getTranslation()).getAngle();
		Rotation2d turretAngleRobot = turretAngleField.minus(robotAngle);
		double hoodAngle = Math.toRadians(m_descriptor.getAngleInterpolator().interpolate(offsetedTargetDist, dz));
		double rollerVelocityRPM = m_descriptor.getRPMInterpolator().interpolate(offsetedTargetDist, dz);

		boolean isWithinShotBounds = offsetedTargetDist >= m_descriptor.getMinDistance()
				&& offsetedTargetDist <= m_descriptor.getMaxDistance();

		// Compute exit pose (end of barrel)
		Transform3d hoodTransform = new Transform3d(
				new Translation3d(
						m_descriptor.getBarrelLength() * Math.cos(hoodAngle) * Math.cos(turretAngleField.getRadians()),
						m_descriptor.getBarrelLength() * Math.cos(hoodAngle) * Math.sin(turretAngleField.getRadians()),
						m_descriptor.getBarrelLength() * Math.sin(hoodAngle)),
				new Rotation3d(0.0, -hoodAngle, turretAngleField.getRadians()));
		Pose3d turretPosePivot = new Pose3d(robotPosition.getX(), robotPosition.getY(), robotPosition3d.getZ(),
				robotPosition3d.getRotation()).transformBy(m_descriptor.getRobotToTurret());
		// Final exit pose after all transforms
		Pose3d exitPose = turretPosePivot.transformBy(hoodTransform);
		// v = w * r * e
		double exitVelocity = Units.rotationsPerMinuteToRadiansPerSecond(rollerVelocityRPM)
				* m_descriptor.getRollerRadius() * projectileType.getECoeff();

		// Vector values
		double hx = hoodTransform.getX();
		double hy = hoodTransform.getY();
		double hz = hoodTransform.getZ();
		double mag = Math.sqrt(hx * hx + hy * hy + hz * hz);

		Vector3 velVec3;
		if (mag >= EPSILON) {
			// Use simple transform vector, normalize, scale by velocity
			velVec3 = new Vector3(hx * exitVelocity / mag, hy * exitVelocity / mag, hz * exitVelocity / mag);
		} else {
			// Convert rotation into vector, normalize, scale by velocity
			double tRad = turretAngleField.getRadians();
			double dirX = Math.cos(hoodAngle) * Math.cos(tRad);
			double dirY = Math.cos(hoodAngle) * Math.sin(tRad);
			double dirZ = Math.sin(hoodAngle);
			double dmag = Math.sqrt(dirX * dirX + dirY * dirY + dirZ * dirZ);

			if (dmag < EPSILON) velVec3 = new Vector3(0.0, 0.0, 0.0);
			else velVec3 = new Vector3(dirX * exitVelocity / dmag, dirY * exitVelocity / dmag,
					dirZ * exitVelocity / dmag);
		}

		// Check angle bounds
		if ((turretAngleRobot.getDegrees() < m_descriptor.getMinTurretAngle()
				&& Math.abs(m_descriptor.getMinTurretAngle()) < 180)
				|| (turretAngleRobot.getDegrees() > m_descriptor.getMaxTurretAngle()
						&& Math.abs(m_descriptor.getMaxTurretAngle()) < 180))
			isWithinShotBounds = false;

		// Add turret robot linear velocity contribution to x,y components
		// (field-relative).
		velVec3 = new Vector3(velVec3.x + vt_x, velVec3.y + vt_y, velVec3.z);

		Logger.recordOutput("ShooterCalculator/TurretAngleRobot", turretAngleRobot);
		Logger.recordOutput("ShooterCalculator/HoodAngleDeg", Math.toDegrees(hoodAngle));

		// Finally return the updated shot parameters
		return new ShooterParameters(isWithinShotBounds, turretAngleField, turretAngleRobot, Math.toDegrees(hoodAngle),
				rollerVelocityRPM, timeOfFlight, exitPose.getTranslation(), velVec3, velVec3.norm());
	}

	/**
	 * Instead of using ideal parameters, computes the actual exit parameters based on current
	 * shooter state. Use this only for simulation purposes. This will show how the pre-calculated
	 * and actual parameters that are achieved with current subsystem states differ.
	 *
	 * @param parameters          The current shooter parameters.
	 * @param hoodAngleDeg        The current hood angle, in degrees.
	 * @param turretAngleRobotDeg The current turret angle relative to the robot, in degrees (can
	 *                            just read encoder values).
	 * @param rollerRPM           The current roller speeds, in RPM.
	 * @param robotPose           The robot's current pose.
	 * @param currentSpeeds       The robot's current chassis speeds.
	 * @param projectileType      The type of projectile being fired.
	 * @return The computed simulation parameters.
	 */
	public ShooterParameters computeCurrentExitParameters(ShooterParameters parameters, double hoodAngleDeg,
			double turretAngleRobotDeg, double rollerRPM, Pose3d robotPose, ChassisSpeeds currentSpeeds,
			ShooterProjectileType projectileType) {
		double hoodAngle = Math.toRadians(hoodAngleDeg);
		Rotation2d turretAngleField = Rotation2d.fromDegrees(turretAngleRobotDeg)
				.plus(robotPose.getRotation().toRotation2d());
		Transform3d hoodTransform = new Transform3d(
				new Translation3d(
						m_descriptor.getBarrelLength() * Math.cos(hoodAngle) * Math.cos(turretAngleField.getRadians()),
						m_descriptor.getBarrelLength() * Math.cos(hoodAngle) * Math.sin(turretAngleField.getRadians()),
						m_descriptor.getBarrelLength() * Math.sin(hoodAngle)),
				new Rotation3d(0.0, -hoodAngle, turretAngleField.getRadians()));
		Pose3d turretPosePivot = robotPose.transformBy(m_descriptor.getRobotToTurret());
		// Final exit pose after all transforms
		Pose3d exitPose = turretPosePivot.transformBy(hoodTransform);
		// v = w * r * e
		double exitVelocity = Units.rotationsPerMinuteToRadiansPerSecond(rollerRPM) * m_descriptor.getRollerRadius()
				* projectileType.getECoeff();

		// Vector values
		double hx = hoodTransform.getX();
		double hy = hoodTransform.getY();
		double hz = hoodTransform.getZ();
		double mag = Math.sqrt(hx * hx + hy * hy + hz * hz);

		Vector3 velVec3;
		if (mag >= EPSILON) {
			// Use simple transform vector, normalize, scale by velocity
			velVec3 = new Vector3(hx * exitVelocity / mag, hy * exitVelocity / mag, hz * exitVelocity / mag);
		} else {
			// Convert rotation into vector, normalize, scale by velocity
			double tRad = turretAngleField.getRadians();
			double dirX = Math.cos(hoodAngle) * Math.cos(tRad);
			double dirY = Math.cos(hoodAngle) * Math.sin(tRad);
			double dirZ = Math.sin(hoodAngle);
			double dmag = Math.sqrt(dirX * dirX + dirY * dirY + dirZ * dirZ);

			if (dmag < EPSILON) velVec3 = new Vector3(0.0, 0.0, 0.0);
			else velVec3 = new Vector3(dirX * exitVelocity / dmag, dirY * exitVelocity / dmag,
					dirZ * exitVelocity / dmag);
		}

		double robotAngle = robotPose.getRotation().getZ();
		double vt_x = currentSpeeds.vxMetersPerSecond
				+ currentSpeeds.omegaRadiansPerSecond * (m_descriptor.getRobotToTurret().getY() * Math.cos(robotAngle)
						- m_descriptor.getRobotToTurret().getX() * Math.sin(robotAngle));
		double vt_y = currentSpeeds.vyMetersPerSecond
				+ currentSpeeds.omegaRadiansPerSecond * (m_descriptor.getRobotToTurret().getX() * Math.cos(robotAngle)
						- m_descriptor.getRobotToTurret().getY() * Math.sin(robotAngle));

		velVec3 = new Vector3(velVec3.x + vt_x, velVec3.y + vt_y, velVec3.z);

		return new ShooterParameters(parameters.isValid, turretAngleField, Rotation2d.fromDegrees(turretAngleRobotDeg),
				hoodAngleDeg, rollerRPM, 0, exitPose.getTranslation(), velVec3, exitVelocity);
	}

	/**
	 * Updates the projectile trajectory visualizer using AdvantageScope. Logs the values to the
	 * given logger key. If performance intensive or if more precision is desired, move to a
	 * Notifier thread.
	 *
	 * @param loggerKey The key to put the projectiles' positions into.
	 * @param list      The list of projectiles to simulate.
	 * @return The updated list, removing all ended projectiles.
	 */
	public List<ShooterProjectile> updateVisualizer(String loggerKey, List<ShooterProjectile> list) {
		List<Pose3d> poses = new ArrayList<>();
		List<ShooterProjectile> newProjectiles = new ArrayList<>();
		for (ShooterProjectile projectile : list) {
			double now = Timer.getFPGATimestamp();
			// Timer not initialized, first run
			if (projectile.lastUpdate == -1) { projectile.lastUpdate = now; projectile.shotAt = now; }
			// No physics updates for frozen projectiles
			if (projectile.isFrozen) { newProjectiles.add(projectile); poses.add(projectile.pose); continue; }
			double dt = now - projectile.lastUpdate;
			// Compute new state
			// Linear approximation (fine at high freq updates): dx = v * t + 1/2 at^2
			Vector3 dx = projectile.velocity.scale(dt).add(projectile.acceleration.scale(0.5 * dt * dt));
			Pose3d lastPose = projectile.pose; // Save pose
			projectile.pose = new Pose3d(projectile.pose.getX() + dx.x, projectile.pose.getY() + dx.y,
					projectile.pose.getZ() + dx.z, rotationFromVelocity(projectile.velocity));
			projectile.velocity = projectile.velocity.add(projectile.acceleration.scale(dt));
			// Check target hits
			boolean destroy = false;
			for (ProjectileTarget target : ProjectileTargets) {
				// We use 2 poses to check for hit, for accuracy purposes
				boolean hit = target.checkHit(lastPose, projectile.pose);
				if (hit && !target.hideProjectileOnHit) {
					projectile.isFrozen = true;
					break;
				} else if (hit && target.hideProjectileOnHit) { destroy = true; break; }
			}
			// Just skip if set to remove
			if (destroy) continue;
			// End condition
			if (projectile.pose.getZ() >= 0 && now - projectile.shotAt <= TrajectoryMaxTime) {
				poses.add(projectile.pose);
				newProjectiles.add(projectile);
			}
			// Update new state
			Vector3 v = projectile.velocity;
			double k = projectile.getType().getDragForceCoeff();
			// -F_net = 1/2 * Cd * rho * A * v * |v| + mg -> (grav + drag)
			Vector3 force = v.scale(k * v.norm())
					.add(new Vector3(0, 0, PhysicsUtil.kG * projectile.getType().getMass()));
			// a = F/m
			Vector3 accel_new = force.scale(-1 / projectile.getType().getMass());
			projectile.acceleration = accel_new;
			projectile.lastUpdate = now;
		}
		Logger.recordOutput(loggerKey, poses.toArray(new Pose3d[poses.size()]));
		return newProjectiles;
	}

	// Utilities //

	/**
	 * Makes the projectile look in the same direction as their velocity and applies random roll
	 * shifting.
	 */
	private Rotation3d rotationFromVelocity(Vector3 v) {
		if (v.norm() < 1e-6) {
			// no movement, identity rotation with a little roll noise
			double rollNoise = randomRoll();
			return new Rotation3d(rollNoise, 0.0, 0.0);
		}
		double yaw = Math.atan2(v.y, v.x);
		double pitch = -Math.atan2(v.z, Math.hypot(v.x, v.y));
		double roll = randomRoll();

		// Rotation3d(roll, pitch, yaw)
		return new Rotation3d(roll, pitch, yaw);
	}

	/** Creates a random X (roll) rotation */
	private double randomRoll() {
		return Math.random() * (2 * MaxRollNoiseRad) - MaxRollNoiseRad;
	}
}
