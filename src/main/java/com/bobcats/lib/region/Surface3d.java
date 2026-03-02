package com.bobcats.lib.region;

import com.bobcats.lib.utils.Utils;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Represents a finite quadrilateral surface in 3D defined by four corner points.
 */
public class Surface3d {
	private final Translation3d m_p1;
	private final Translation3d m_p2;
	private final Translation3d m_p3;
	private final Translation3d m_p4;

	private final Translation3d m_normal;

	/**
	 * Constructs a new Surface3d.
	 * <p>
	 * Validates that the first three points are non-collinear and that the fourth point is
	 * coplanar. The provided points must be in order.
	 *
	 * @param p1 The first corner.
	 * @param p2 The second corner.
	 * @param p3 The third corner.
	 * @param p4 The fourth corner.
	 * @throws IllegalArgumentException If points are invalid or non-coplanar
	 */
	public Surface3d(Translation3d p1, Translation3d p2, Translation3d p3, Translation3d p4) {
		m_p1 = p1;
		m_p2 = p2;
		m_p3 = p3;
		m_p4 = p4;

		// Compute two edge vectors
		Translation3d v1 = p2.minus(p1);
		Translation3d v2 = p3.minus(p1);

		// Compute cross product manually
		double cx = v1.getY() * v2.getZ() - v1.getZ() * v2.getY();
		double cy = v1.getZ() * v2.getX() - v1.getX() * v2.getZ();
		double cz = v1.getX() * v2.getY() - v1.getY() * v2.getX();
		// Ensure first three points are not collinear or duplicates
		double crossMag = Math.sqrt(cx * cx + cy * cy + cz * cz);
		if (crossMag < Utils.EPSILON)
			throw new IllegalArgumentException("Points p1, p2, p3 are collinear or duplicates");

		// Normalize to create plane normal
		m_normal = new Translation3d(cx / crossMag, cy / crossMag, cz / crossMag);

		// Validate that p4 is coplanar: check dot((p4-p1), normal) == 0
		Translation3d v4 = p4.minus(p1);
		double coplanarity = v4.getX() * m_normal.getX() + v4.getY() * m_normal.getY() + v4.getZ() * m_normal.getZ();
		if (Math.abs(coplanarity) > Utils.EPSILON)
			throw new IllegalArgumentException("Point p4 is not coplanar with the first three points");
	}

	/**
	 * Tests if a single pose lies within the finite quad surface (including edges).
	 *
	 * @param testPose The pose to test.
	 * @return true if the point is on the plane.
	 */
	public boolean isColliding(Pose3d testPose) {
		Translation3d point = testPose.getTranslation();
		double dist = signedDistanceToPlane(point);
		if (Math.abs(dist) > Utils.EPSILON) return false;
		// Project onto plane and test triangles
		Translation3d proj = point.minus(m_normal.times(dist));
		return pointInTriangle(proj, m_p1, m_p2, m_p3) || pointInTriangle(proj, m_p1, m_p3, m_p4);
	}

	/**
	 * Tests if the linear motion from lastPose to currentPose intersects the finite quad.
	 *
	 * @param lastPose    The starting pose.
	 * @param currentPose The ending pose.
	 * @return true if the segment crosses or touches the quad.
	 */
	public boolean isColliding(Pose3d lastPose, Pose3d currentPose) {
		Translation3d a = lastPose.getTranslation();
		Translation3d b = currentPose.getTranslation();
		double dA = signedDistanceToPlane(a);
		double dB = signedDistanceToPlane(b);

		// If both endpoints on same side and not touching plane, no intersection
		if (dA * dB > 0 && Math.abs(dA) > Utils.EPSILON && Math.abs(dB) > Utils.EPSILON) return false;

		// Compute intersection point along segment
		double t = (Math.abs(dA) < Utils.EPSILON) ? 0.0 : dA / (dA - dB);
		Translation3d intersect = a.plus(b.minus(a).times(t));

		// Check if intersection is inside quad
		return pointInTriangle(intersect, m_p1, m_p2, m_p3) || pointInTriangle(intersect, m_p1, m_p3, m_p4);
	}

	// Signed distance from point to the plane
	private double signedDistanceToPlane(Translation3d point) {
		Translation3d v = point.minus(m_p1);
		return v.getX() * m_normal.getX() + v.getY() * m_normal.getY() + v.getZ() * m_normal.getZ();
	}

	private boolean pointInTriangle(Translation3d pt, Translation3d a, Translation3d b, Translation3d c) {
		// Edge vectors
		Translation3d v0 = c.minus(a);
		Translation3d v1 = b.minus(a);
		Translation3d v2 = pt.minus(a);

		// Dot products
		double dot00 = v0.getX() * v0.getX() + v0.getY() * v0.getY() + v0.getZ() * v0.getZ();
		double dot01 = v0.getX() * v1.getX() + v0.getY() * v1.getY() + v0.getZ() * v1.getZ();
		double dot02 = v0.getX() * v2.getX() + v0.getY() * v2.getY() + v0.getZ() * v2.getZ();
		double dot11 = v1.getX() * v1.getX() + v1.getY() * v1.getY() + v1.getZ() * v1.getZ();
		double dot12 = v1.getX() * v2.getX() + v1.getY() * v2.getY() + v1.getZ() * v2.getZ();

		double denom = dot00 * dot11 - dot01 * dot01;
		if (Math.abs(denom) < Utils.EPSILON) return false;
		double invDenom = 1.0 / denom;
		double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
		double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

		return (u >= -Utils.EPSILON) && (v >= -Utils.EPSILON) && (u + v <= 1 + Utils.EPSILON);
	}
}
