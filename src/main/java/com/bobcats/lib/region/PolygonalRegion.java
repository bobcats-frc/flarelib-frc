package com.bobcats.lib.region;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.dyn4j.geometry.Convex;

/**
 * A polygonal region on the field. Used for dividing the field into smaller sections.
 *
 * The vertices must be specified in (either CW or CCW) order, and the polygon must be
 * non-self-intersecting.
 *
 * @deprecated Use {@link Convex} instead.
 */
@Deprecated(forRemoval = false)
public class PolygonalRegion {
	private final List<Translation2d> m_vertices;

	/**
	 * Constructs a new PolygonalRegion.
	 *
	 * @param vertices The ordered list of corner points (A, B, C, ...). Must form a simple
	 *                 (non-self-intersecting) polygon.
	 */
	public PolygonalRegion(List<Translation2d> vertices) {
		if (vertices == null || vertices.size() < 3) {
			throw new IllegalArgumentException("must have at least 3 vertices");
		}
		m_vertices = List.copyOf(vertices);
	}

	// Public methods //

	/**
	 * Returns the list of vertices in order.
	 *
	 * @return A list of Translation2d corners.
	 */
	public List<Translation2d> getVertices() { return m_vertices; }

	/**
	 * Returns whether the given pose (x, y) lies within this polygon.
	 *
	 * @param pose The pose to check.
	 * @return True if the pose's (x, y) is inside or on the boundary.
	 */
	public boolean isPoseWithinArea(Pose2d pose) {
		return containsPoint(new Translation2d(pose.getX(), pose.getY()));
	}

	/**
	 * Returns whether the given 2D point lies within this polygon (including boundary). Uses the
	 * ray-casting (crossing-number) algorithm.
	 *
	 * @param point The point to test.
	 * @return True if the point is inside or on an edge.
	 */
	public boolean containsPoint(Translation2d point) {
		double x = point.getX();
		double y = point.getY();

		// Ray-casting: cast a horizontal ray to +infinity in X, count crossings.
		boolean inside = false;
		int n = m_vertices.size();

		// Check if point lies on any edge (boundary)
		for (int i = 0; i < n; i++) {
			Translation2d A = m_vertices.get(i);
			Translation2d B = m_vertices.get((i + 1) % n);
			if (pointOnEdge(point, A, B)) { return true; }
		}

		for (int i = 0; i < n; i++) {
			Translation2d A = m_vertices.get(i);
			Translation2d B = m_vertices.get((i + 1) % n);

			double ax = A.getX();
			double ay = A.getY();
			double bx = B.getX();
			double by = B.getY();

			// Check if the horizontal ray at y crosses edge (A->B)
			boolean intersects = ((ay > y) != (by > y)) // one vertex above, one below
					&& (x < (bx - ax) * (y - ay) / (by - ay) + ax);
			if (intersects) { inside = !inside; }
		}
		return inside;
	}

	/**
	 * Test whether this polygon collides (overlaps) with another polygon.
	 *
	 * @param other The other PolygonalRegion to test against.
	 * @return True if they overlap at all.
	 */
	public boolean collidesWith(PolygonalRegion other) {
		// Check edge intersections
		int n1 = m_vertices.size();
		int n2 = other.getVertices().size();

		for (int i = 0; i < n1; i++) {
			Translation2d p1 = m_vertices.get(i);
			Translation2d p2 = m_vertices.get((i + 1) % n1);

			for (int j = 0; j < n2; j++) {
				Translation2d q1 = other.getVertices().get(j);
				Translation2d q2 = other.getVertices().get((j + 1) % n2);

				if (linesIntersect(p1, p2, q1, q2)) { return true; }
			}
		}

		// Check any vertex of this polygon inside other
		for (Translation2d vertex : m_vertices) { if (other.containsPoint(vertex)) { return true; } }

		// Check any vertex of other polygon inside this
		for (Translation2d vertex : other.getVertices()) { if (containsPoint(vertex)) { return true; } }

		// Otherwise, no collision.
		return false;
	}

	// Private helper methods //

	/**
	 * Determines if the line segment (p1->p2) intersects the segment (p3->p4). Uses orientation
	 * tests.
	 */
	private static boolean linesIntersect(Translation2d p1, Translation2d p2, Translation2d p3, Translation2d p4) {
		double x1 = p1.getX(), y1 = p1.getY();
		double x2 = p2.getX(), y2 = p2.getY();
		double x3 = p3.getX(), y3 = p3.getY();
		double x4 = p4.getX(), y4 = p4.getY();

		// Compute orientations
		int o1 = orientation(x1, y1, x2, y2, x3, y3);
		int o2 = orientation(x1, y1, x2, y2, x4, y4);
		int o3 = orientation(x3, y3, x4, y4, x1, y1);
		int o4 = orientation(x3, y3, x4, y4, x2, y2);

		// General case: segments straddle each other
		if (o1 != o2 && o3 != o4) { return true; }

		// Special cases: collinear & overlapping
		if (o1 == 0 && onSegment(x1, y1, x2, y2, x3, y3)) return true;
		if (o2 == 0 && onSegment(x1, y1, x2, y2, x4, y4)) return true;
		if (o3 == 0 && onSegment(x3, y3, x4, y4, x1, y1)) return true;
		if (o4 == 0 && onSegment(x3, y3, x4, y4, x2, y2)) return true;

		return false;
	}

	/**
	 * Compute the orientation of the triplet (x1,y1), (x2,y2), (x3,y3).
	 *
	 * @return 0 if collinear, 1 if clockwise, 2 if counterclockwise.
	 */
	private static int orientation(double x1, double y1, double x2, double y2, double x3, double y3) {
		double val = (y2 - y1) * (x3 - x2) - (x2 - x1) * (y3 - y2);
		if (Math.abs(val) < 1e-9) {
			return 0; // collinear
		}
		return (val > 0) ? 1 : 2; // 1 = clockwise, 2 = counterclockwise
	}

	/**
	 * Check if point (x3,y3) lies on the segment (x1,y1)-(x2,y2).
	 */
	private static boolean onSegment(double x1, double y1, double x2, double y2, double x3, double y3) {
		return x3 <= Math.max(x1, x2) && x3 >= Math.min(x1, x2) && y3 <= Math.max(y1, y2) && y3 >= Math.min(y1, y2);
	}

	/**
	 * Check if point p lies on the edge defined by points A and B.
	 */
	private boolean pointOnEdge(Translation2d p, Translation2d A, Translation2d B) {
		double x1 = A.getX(), y1 = A.getY();
		double x2 = B.getX(), y2 = B.getY();
		double x = p.getX(), y = p.getY();

		if (orientation(x1, y1, x2, y2, x, y) != 0) return false;
		return onSegment(x1, y1, x2, y2, x, y);
	}
}
