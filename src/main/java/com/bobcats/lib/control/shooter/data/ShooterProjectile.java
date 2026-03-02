package com.bobcats.lib.control.shooter.data;

import com.bobcats.lib.container.Vector3;
import com.bobcats.lib.math.PhysicsUtil;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.UUID;

/**
 * A class containing physical parameters for a projectile shot from the shooter.
 */
public class ShooterProjectile {
	private ShooterProjectileType m_type;
	private String m_id;
	public Vector3 velocity;
	public Vector3 acceleration;
	public Pose3d pose;
	public double lastUpdate = -1;
	public double shotAt;
	public boolean isFrozen;

	/**
	 * Constructs a new ShooterProjectile.
	 *
	 * @param type            The shooter projectile type.
	 * @param initialVelocity The initial shooting velocity.
	 * @param startingPose    The starting pose, the end of the barrel.
	 */
	public ShooterProjectile(ShooterProjectileType type, Vector3 initialVelocity, Pose3d startingPose) {
		m_type = type;
		velocity = initialVelocity;
		pose = startingPose;
		m_id = UUID.randomUUID().toString();

		// Get initial acceleration
		double speed = Math.sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z);
		Vector3 force = velocity.scale(m_type.getDragForceCoeff() * speed)
				.add(new Vector3(0, 0, PhysicsUtil.kG * type.getMass()));
		acceleration = force.scale(-1 / type.getMass());
	}

	public ShooterProjectileType getType() { return m_type; }

	public String getId() { return m_id; }
}
