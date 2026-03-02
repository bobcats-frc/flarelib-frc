package com.bobcats.lib.control.shooter.data;

import com.bobcats.lib.region.Surface3d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;

/** A class representing a projectile target on the field. */
public class ProjectileTarget {
	private Translation3d m_point;
	private double m_tolerance;

	private Surface3d m_surface;

	public final boolean hideProjectileOnHit;

	/**
	 * Constructs a new ProjectileTarget.
	 *
	 * @param surface             The hit surface.
	 * @param hideProjectileOnHit Whether to destroy the projectile after hit.
	 */
	public ProjectileTarget(Surface3d surface, boolean hideProjectileOnHit) {
		m_surface = surface;
		this.hideProjectileOnHit = hideProjectileOnHit;
	}

	/**
	 * Constructs a new ProjectileTarget.
	 *
	 * @param point               The target point.
	 * @param toleranceMeters     The tolerance for the hit, in meters.
	 * @param hideProjectileOnHit Whether to destroy the projectile after hit.
	 */
	public ProjectileTarget(Translation3d point, double toleranceMeters, boolean hideProjectileOnHit) {
		m_point = point;
		m_tolerance = toleranceMeters;
		this.hideProjectileOnHit = hideProjectileOnHit;
	}

	/**
	 * Checks whether the target was hit or not over a very short interval.
	 *
	 * @param prev    The previous pose.
	 * @param current The current pose.
	 * @return True if hit, false otherwise.
	 */
	public boolean checkHit(Pose3d prev, Pose3d current) {
		if (m_surface != null) {
			// Surface3d exists, check collision
			return m_surface.isColliding(prev, current);
		} else {
			// Get midpoint
			Pose3d mid = prev.interpolate(current, 0.5);
			// Check tolerance
			return MathUtil.isNear(0, mid.getTranslation().getDistance(m_point), m_tolerance);
		}
	}
}
