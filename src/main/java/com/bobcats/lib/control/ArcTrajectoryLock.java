package com.bobcats.lib.control;
// Copyright 2025-2027 Bobcats Robotics
// GitHub https://github.com/bobcats-frc
// This project is included under an MIT license by the LICENSE file located at
// the root project folder.
import com.bobcats.lib.utils.AllianceUtil;
import com.bobcats.lib.utils.Utils;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.Optional;

/**
 * A utility class for a swerve-lock trajectory arc designed for teleop shooting.
 */
public final class ArcTrajectoryLock {
	private final Translation2d m_target;
	private final double m_radius;
	private final double m_radiusKp, m_velocityKa;

	// Angular boundaries for partial arcs
	private final Optional<Rotation2d> m_minAngle;
	private final Optional<Rotation2d> m_maxAngle;

	private boolean m_autoFlipTarget = false;

	/**
	 * The buffer angle at the end of partial arcs for position limiting, in radians.
	 */
	public double PartialArcEndpointBufferRad = Math.toRadians(4.0);

	/**
	 * Constructs a new ArcTrajectoryLock.
	 *
	 * @param target     The field target translation.
	 * @param radius     The allowed align distance.
	 * @param radiusKp   The kP coefficient for radial correction.
	 * @param velocityKa The amplification constant for the sine when sine scaling the tangential
	 *                   velocity. Put 1 for default. <br>
	 *                   <code> |v_t| = bound(kA * Math.sin(clampedError), -1, 1) * |v_desired| </code>
	 */
	public ArcTrajectoryLock(Translation2d target, double radius, double radiusKp, double velocityKa) {
		this(target, radius, radiusKp, velocityKa, Optional.empty(), Optional.empty());
	}

	/**
	 * Constructs a new ArcTrajectoryLock.
	 *
	 * <p>
	 * Creates a partial arc with the given angle bounds, in WPILib field-relative conventions.
	 *
	 * @param target     The field target translation.
	 * @param radius     The allowed align distance.
	 * @param radiusKp   The kP coefficient for radial correction.
	 * @param velocityKa The amplification constant for the sine when sine scaling the tangential
	 *                   velocity. Put 1 for default. <br>
	 *                   <code> |v_t| = bound(kA * Math.sin(clampedError), -1, 1) * |v_desired| </code>
	 * @param minAngle   The minimum angle of the arc, following WPILib conventions.
	 * @param maxAngle   The maximum angle of the arc, following WPILib conventions.
	 */
	public ArcTrajectoryLock(Translation2d target, double radius, double radiusKp, double velocityKa,
			Rotation2d minAngle, Rotation2d maxAngle) {
		this(target, radius, radiusKp, velocityKa, Optional.of(minAngle), Optional.of(maxAngle));
	}

	/** Constructs a new ArcTrajectoryLock. */
	private ArcTrajectoryLock(Translation2d target, double radius, double radiusKp, double velocityKa,
			Optional<Rotation2d> minAngle, Optional<Rotation2d> maxAngle) {
		if (radius <= 0 || radiusKp <= 0 || velocityKa <= 0 || target == null)
			throw new IllegalArgumentException("invalid parameters provided to ArcTrajectoryLock");
		m_target = target;
		m_radius = radius;
		m_radiusKp = radiusKp;
		m_velocityKa = velocityKa;
		m_minAngle = minAngle;
		m_maxAngle = maxAngle;
	}

	/**
	 * Returns the point closest to the robot on the arc.
	 *
	 * @param robotPose The current measured field robot pose.
	 * @return The closest point on the arc.
	 */
	public Pose2d getClosestValidArcPoint(Pose2d robotPose) {
		Translation2d p = robotPose.getTranslation();
		Translation2d rel = p.minus(getTarget());
		double r = rel.getNorm();

		// Clamp angle if bounded
		Rotation2d angle = rel.getAngle();
		if (r < Utils.EPSILON) angle = robotPose.getRotation();
		if (m_minAngle.isPresent() && m_maxAngle.isPresent()) {
			double angleRad = MathUtil.angleModulus(angle.getRadians());
			double minRad = MathUtil.angleModulus(flipAngle(m_minAngle.get()).getRadians());
			double maxRad = MathUtil.angleModulus(flipAngle(m_maxAngle.get()).getRadians());

			// Basic wrap-around check for angular clamping
			angle = Rotation2d.fromRadians(clampAngle(angleRad, minRad, maxRad));
		}

		Translation2d projected = getTarget().plus(new Translation2d(m_radius, angle));
		Rotation2d facingTarget = getTarget().minus(projected).getAngle();

		return new Pose2d(projected, facingTarget);
	}

	/**
	 * Corrects the given desired chassis speeds to follow the trajectory arc with the given
	 * velocity limit.
	 *
	 * @param robotPose     The current measured field robot pose.
	 * @param desiredSpeeds The desired field-relative velocity, usually obtained from joysticks.
	 * @param maxVelocity   The maximum allowed robot velocity, in m/s.
	 * @return The corrected field-relative chassis speeds.
	 */
	public ChassisSpeeds getCorrectedTrajectorySpeeds(Pose2d robotPose, ChassisSpeeds desiredSpeeds,
			double maxVelocity) {
		Translation2d p = robotPose.getTranslation();
		Translation2d rel = p.minus(getTarget());
		double r = rel.getNorm();

		if (r < Utils.EPSILON) {
			Translation2d velocity = new Translation2d(desiredSpeeds.vxMetersPerSecond,
					desiredSpeeds.vyMetersPerSecond);
			// Apply max velocity
			double limitFactor = maxVelocity / Math.max(velocity.getNorm(), maxVelocity);
			velocity = velocity.times(limitFactor);
			return new ChassisSpeeds(velocity.getX(), velocity.getY(), desiredSpeeds.omegaRadiansPerSecond);
		}

		// Unit radius and tangent vectors
		Translation2d ur = rel.times(1.0 / r);
		Translation2d ut = new Translation2d(-ur.getY(), ur.getX());

		Translation2d v = new Translation2d(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond);

		// Tangential velocitiy
		double vt = 0;
		if (v.getNorm() > Utils.EPSILON) {
			Rotation2d headingAngle = new Rotation2d(v.getX(), v.getY());
			Rotation2d robotAngle = rel.getAngle();

			double angleError = headingAngle.minus(robotAngle).getRadians();
			// Clamp sine between the values it reached the min, max values
			double clampedError = MathUtil.clamp(angleError, -Math.PI / 2.0, Math.PI / 2.0);
			// The sine curve smoothly decelerates the robot
			vt = MathUtil.clamp(m_velocityKa * Math.sin(clampedError), -1.0, 1.0) * v.getNorm();
		}

		// Apply boundary constraints
		if (m_minAngle.isPresent() && m_maxAngle.isPresent()) {
			double currentAngle = MathUtil.angleModulus(rel.getAngle().getRadians());
			double min = MathUtil.angleModulus(flipAngle(m_minAngle.get()).getRadians());
			double max = MathUtil.angleModulus(flipAngle(m_maxAngle.get()).getRadians());

			// If at the CCW boundary and trying to go CCW, kill vt
			if (isAtBoundary(currentAngle, max) && vt > 0) vt = 0;
			// If at the CW boundary and trying to go CW, kill vt
			if (isAtBoundary(currentAngle, min) && vt < 0) vt = 0;
		}

		// Tangential velocity
		Translation2d tangentialVelocity = ut.times(vt);

		// Radial velocity
		Pose2d closestArcPoint = getClosestValidArcPoint(robotPose);
		Translation2d targetTranslation = closestArcPoint.getTranslation();
		Translation2d radialCorrectionVelocity = targetTranslation.minus(p).times(m_radiusKp);

		// Combined velocity
		Translation2d correctedFieldVelocity = tangentialVelocity.plus(radialCorrectionVelocity);

		// Limit velocity
		if (correctedFieldVelocity.getNorm() > maxVelocity) {
			double sqDiff = Math.max(0, maxVelocity * maxVelocity
					- radialCorrectionVelocity.getNorm() * radialCorrectionVelocity.getNorm());

			tangentialVelocity = ut.times(Math.sqrt(sqDiff) * Math.signum(vt));
			correctedFieldVelocity = tangentialVelocity.plus(radialCorrectionVelocity);

			// In case radial correction is greater than the limit
			double correctedMag = correctedFieldVelocity.getNorm();
			if (correctedMag > maxVelocity) {
				double correctiveFactor = maxVelocity / Math.max(correctedMag, maxVelocity);
				correctedFieldVelocity = correctedFieldVelocity.times(correctiveFactor);
			}
		}

		return new ChassisSpeeds(correctedFieldVelocity.getX(), correctedFieldVelocity.getY(),
				desiredSpeeds.omegaRadiansPerSecond);
	}

	/**
	 * Corrects the given desired chassis speeds to follow the trajectory arc with the given
	 * velocity limit.
	 *
	 * @param robotPose                   The current measured field robot pose.
	 * @param desiredSpeeds               The desired field-relative velocity, usually obtained
	 *                                    from joysticks.
	 * @param maxTangentialVelocity       The maximum allowed tangential robot velocity, in m/s.
	 * @param maxRadialCorrectionVelocity The maximum allowed radial correction robot velocity, in
	 *                                    m/s.
	 * @param maxVelocity                 The maximum allowed robot velocity, in m/s.
	 * @return The corrected field-relative chassis speeds.
	 */
	public ChassisSpeeds getCorrectedTrajectorySpeeds(Pose2d robotPose, ChassisSpeeds desiredSpeeds,
			double maxTangentialVelocity, double maxRadialCorrectionVelocity, double maxVelocity) {
		Translation2d p = robotPose.getTranslation();
		Translation2d rel = p.minus(getTarget());
		double r = rel.getNorm();

		if (r < Utils.EPSILON) {
			Translation2d velocity = new Translation2d(desiredSpeeds.vxMetersPerSecond,
					desiredSpeeds.vyMetersPerSecond);
			// Apply max velocity
			double limitFactor = maxVelocity / Math.max(velocity.getNorm(), maxVelocity);
			velocity = velocity.times(limitFactor);
			return new ChassisSpeeds(velocity.getX(), velocity.getY(), desiredSpeeds.omegaRadiansPerSecond);
		}

		Translation2d ur = rel.times(1.0 / r);
		Translation2d ut = new Translation2d(-ur.getY(), ur.getX());

		Translation2d v = new Translation2d(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond);

		// Tangential velocitiy
		double vt = 0;
		if (v.getNorm() > Utils.EPSILON) {
			Rotation2d stickAngle = new Rotation2d(v.getX(), v.getY());
			Rotation2d robotAngle = rel.getAngle();

			double angleError = stickAngle.minus(robotAngle).getRadians();
			// Clamp angle between the the min, max sine values
			double clampedError = MathUtil.clamp(angleError, -Math.PI / 2.0, Math.PI / 2.0);
			// The sine curve smoothly decelerates the robot
			vt = MathUtil.clamp(m_velocityKa * Math.sin(clampedError), -1.0, 1.0) * v.getNorm();
		}

		// Apply boundary constraints
		if (m_minAngle.isPresent() && m_maxAngle.isPresent()) {
			double currentAngle = MathUtil.angleModulus(rel.getAngle().getRadians());
			double min = MathUtil.angleModulus(flipAngle(m_minAngle.get()).getRadians());
			double max = MathUtil.angleModulus(flipAngle(m_maxAngle.get()).getRadians());

			// If at the CCW boundary and trying to go CCW, kill vt
			if (isAtBoundary(currentAngle, max) && vt > 0) vt = 0;
			// If at the CW boundary and trying to go CW, kill vt
			if (isAtBoundary(currentAngle, min) && vt < 0) vt = 0;
		}

		// Calculate and limit velocities
		Translation2d tangentialVelocity = ut.times(vt);
		double tangentialCorrectiveFactor = maxTangentialVelocity
				/ Math.max(tangentialVelocity.getNorm(), maxTangentialVelocity);
		tangentialVelocity = tangentialVelocity.times(tangentialCorrectiveFactor);

		// Radial velocity
		Pose2d closestArcPoint = getClosestValidArcPoint(robotPose);
		Translation2d targetTranslation = closestArcPoint.getTranslation();
		Translation2d radialCorrectionVelocity = targetTranslation.minus(p).times(m_radiusKp);

		double radialCorrectiveFactor = maxRadialCorrectionVelocity
				/ Math.max(radialCorrectionVelocity.getNorm(), maxRadialCorrectionVelocity);
		radialCorrectionVelocity = radialCorrectionVelocity.times(radialCorrectiveFactor);

		// Combined velocity
		Translation2d correctedFieldVelocity = tangentialVelocity.plus(radialCorrectionVelocity);

		// Apply final max velocity
		double limitFactor = maxVelocity / Math.max(correctedFieldVelocity.getNorm(), maxVelocity);
		correctedFieldVelocity = correctedFieldVelocity.times(limitFactor);

		return new ChassisSpeeds(correctedFieldVelocity.getX(), correctedFieldVelocity.getY(),
				desiredSpeeds.omegaRadiansPerSecond);
	}

	/**
	 * Sets whether to automatically flip the target based on alliance. The given target must be
	 * blue-alliance coordinates.
	 *
	 * @param flip Whether to automatically flip the target based on alliance.
	 * @return The instance for chaining.
	 */
	public ArcTrajectoryLock withAutoFlipTarget(boolean flip) {
		m_autoFlipTarget = flip;
		return this;
	}

	// Private Helpers //

	/** Clamps the given angle between the 2 others. */
	private double clampAngle(double angle, double min, double max) {
		if (min <= max) {
			return Math.max(min, Math.min(max, angle));
		} else {
			// Handles cases where the arc crosses the PI/-PI boundary
			return (angle >= min || angle <= max) ? angle : (Math.abs(angle - min) < Math.abs(angle - max) ? min : max);
		}
	}

	/** Returns whether the arc boundary was reached. */
	private boolean isAtBoundary(double current, double boundary) {
		return Math.abs(MathUtil.angleModulus(current - boundary)) < PartialArcEndpointBufferRad;
	}

	/** Returns the appropriate target. */
	private Translation2d getTarget() {
		return m_autoFlipTarget ? AllianceUtil.flipWithAlliance(m_target) : m_target;
	}

	/** Returns the appropriately flipped angle. */
	private Rotation2d flipAngle(Rotation2d rot) {
		return m_autoFlipTarget ? AllianceUtil.flipWithAlliance(rot) : rot;
	}
}
