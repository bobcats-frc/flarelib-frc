package com.bobcats.lib.control.shooter.data;

/** Holder class for a shooter projectile. */
public class ShooterProjectileType {

	private final double m_moi;
	private double m_dragFCoeff;
	private final double m_mass;
	private final double m_eCoeff;

	/**
	 * Constructs a new ShooterProjectile.
	 *
	 * @param effectiveMOI         The effective moment of inertia. See {@link ProjectileShape}.
	 * @param dragForceCoefficient The drag force coefficient, obtained by
	 *                             <code>0.5*rho_air*A*Cd</code>.
	 * @param mass                 The mass in kg.
	 * @param e                    The velocity transfer coefficient, to match the actual behavior
	 *                             of the projectile during launch. Fits the formula
	 *                             <code>v_exit = e*v_rollerSurface</code>.
	 */
	public ShooterProjectileType(double effectiveMOI, double dragForceCoefficient, double mass, double e) {
		m_moi = effectiveMOI;
		m_mass = mass;
		m_eCoeff = e;
		m_dragFCoeff = dragForceCoefficient;
	}

	// Getters //

	public double getEffectiveInertia() { return m_moi; }

	public double getDragForceCoeff() { return m_dragFCoeff; }

	public double getMass() { return m_mass; }

	public double getECoeff() { return m_eCoeff; }

	// Setters //

	/**
	 * Sets the drag force coefficient.
	 *
	 * <p>
	 * <b>Tip</b>: If you compute the drag force coefficient with a known rho (air density), such
	 * as 1.225 kg/m^3, then you can multiply by <code>currentAirDensity/1.225</code> to get the
	 * updated value for the current density.
	 *
	 * @param val The new value.
	 */
	public void setDragForceCoeff(double val) { m_dragFCoeff = val; }

	// Classes //

	/**
	 * Represents various projectile shapes with moment of inertia calculations. All moments are
	 * calculated about central axes through center of mass.
	 *
	 * <p>
	 * For clarification and X-Y-Z axes orientation, check out the wikipedia page
	 * https://en.wikipedia.org/wiki/List_of_moments_of_inertia.
	 */
	public static abstract class ProjectileShape {
		/**
		 * Calculates moment of inertia about X-axis.
		 *
		 * @return Moment of inertia about X-axis in kg*m^2.
		 */
		public abstract double inertiaX();

		/**
		 * Calculates moment of inertia about Y-axis.
		 *
		 * @return Moment of inertia about Y-axis in kg*m^2.
		 */
		public abstract double inertiaY();

		/**
		 * Calculates moment of inertia about Z-axis.
		 *
		 * @return Moment of inertia about Z-axis in kg*m^2.
		 */
		public abstract double inertiaZ();

		/**
		 * Solid cylinder. Axis orientation: Z-axis along central axis.
		 */
		public static class CylinderSolid extends ProjectileShape {
			private final double m_mass;
			private final double m_radius;
			private final double m_height;

			/**
			 * Constructs a new CylinderSolid.
			 *
			 * @param mass   Total mass in kg.
			 * @param radius Radius in meters.
			 * @param height Height along Z-axis in meters.
			 */
			public CylinderSolid(double mass, double radius, double height) {
				m_mass = mass;
				m_radius = radius;
				m_height = height;
			}

			@Override
			public double inertiaX() {
				return (1.0 / 12.0) * m_mass * (3 * m_radius * m_radius + m_height * m_height);
			}

			@Override
			public double inertiaY() {
				return inertiaX();
			}

			@Override
			public double inertiaZ() {
				return (1.0 / 2.0) * m_mass * m_radius * m_radius;
			}
		}

		/**
		 * Thin-walled hollow cylinder. Axis orientation: Z-axis along central axis.
		 */
		public static class CylinderHollowThin extends ProjectileShape {
			private final double m_mass;
			private final double m_radius;
			private final double m_height;

			/**
			 * Constructs a new CylinderHollowThin.
			 *
			 * @param mass   Total mass in kg.
			 * @param radius Radius in meters.
			 * @param height Height along Z-axis in meters.
			 */
			public CylinderHollowThin(double mass, double radius, double height) {
				m_mass = mass;
				m_radius = radius;
				m_height = height;
			}

			@Override
			public double inertiaX() {
				return (1.0 / 12.0) * m_mass * (6 * m_radius * m_radius + m_height * m_height);
			}

			@Override
			public double inertiaY() {
				return inertiaX();
			}

			@Override
			public double inertiaZ() {
				return m_mass * m_radius * m_radius;
			}
		}

		/**
		 * Thick-walled hollow cylinder. Axis orientation: Z-axis along central axis.
		 */
		public static class CylinderHollowThick extends ProjectileShape {
			private final double m_mass;
			private final double m_innerRadius;
			private final double m_outerRadius;
			private final double m_height;

			/**
			 * Constructs a new CylinderHollowThick.
			 *
			 * @param mass        Total mass in kg.
			 * @param innerRadius Inner radius in meters.
			 * @param outerRadius Outer radius in meters.
			 * @param height      Height along Z-axis in meters.
			 */
			public CylinderHollowThick(double mass, double innerRadius, double outerRadius, double height) {
				m_mass = mass;
				m_innerRadius = innerRadius;
				m_outerRadius = outerRadius;
				m_height = height;
			}

			@Override
			public double inertiaX() {
				return (1.0 / 12.0) * m_mass
						* (3 * (m_innerRadius * m_innerRadius + m_outerRadius * m_outerRadius) + m_height * m_height);
			}

			@Override
			public double inertiaY() {
				return inertiaX();
			}

			@Override
			public double inertiaZ() {
				return (1.0 / 2.0) * m_mass * (m_innerRadius * m_innerRadius + m_outerRadius * m_outerRadius);
			}
		}

		/**
		 * Solid sphere. Axis orientation: All axes equivalent through center.
		 */
		public static class SphereSolid extends ProjectileShape {
			private final double m_mass;
			private final double m_radius;

			/**
			 * Constructs a new SphereSolid.
			 *
			 * @param mass   Total mass in kg.
			 * @param radius Radius in meters.
			 */
			public SphereSolid(double mass, double radius) {
				m_mass = mass;
				m_radius = radius;
			}

			@Override
			public double inertiaX() {
				return (2.0 / 5.0) * m_mass * m_radius * m_radius;
			}

			@Override
			public double inertiaY() {
				return inertiaX();
			}

			@Override
			public double inertiaZ() {
				return inertiaX();
			}
		}

		/**
		 * Thin-walled hollow sphere. Axis orientation: All axes equivalent through center.
		 */
		public static class SphereHollowThin extends ProjectileShape {
			private final double m_mass;
			private final double m_radius;

			/**
			 * Constructs a new SphereHollowThin.
			 *
			 * @param mass   Total mass in kg.
			 * @param radius Radius in meters.
			 */
			public SphereHollowThin(double mass, double radius) {
				m_mass = mass;
				m_radius = radius;
			}

			@Override
			public double inertiaX() {
				return (2.0 / 3.0) * m_mass * m_radius * m_radius;
			}

			@Override
			public double inertiaY() {
				return inertiaX();
			}

			@Override
			public double inertiaZ() {
				return inertiaX();
			}
		}

		/**
		 * Thick-walled hollow sphere. Axis orientation: All axes equivalent through center.
		 */
		public static class SphereHollowThick extends ProjectileShape {
			private final double m_mass;
			private final double m_innerRadius;
			private final double m_outerRadius;

			/**
			 * Constructs a new SphereHollowThick.
			 *
			 * @param mass        Total mass in kg.
			 * @param innerRadius Inner radius in meters.
			 * @param outerRadius Outer radius in meters.
			 */
			public SphereHollowThick(double mass, double innerRadius, double outerRadius) {
				m_mass = mass;
				m_innerRadius = innerRadius;
				m_outerRadius = outerRadius;
			}

			@Override
			public double inertiaX() {
				return (2.0 / 5.0) * m_mass * (Math.pow(m_outerRadius, 5) - Math.pow(m_innerRadius, 5))
						/ (Math.pow(m_outerRadius, 3) - Math.pow(m_innerRadius, 3));
			}

			@Override
			public double inertiaY() {
				return inertiaX();
			}

			@Override
			public double inertiaZ() {
				return inertiaX();
			}
		}

		/**
		 * Solid cuboid. Axis orientation: X-width, Y-depth, Z-height.
		 */
		public static class CuboidSolid extends ProjectileShape {
			private final double m_mass;
			private final double m_width; // X-dimension
			private final double m_height; // Y-dimension
			private final double m_depth; // Z-dimension

			/**
			 * Constructs a new CuboidSolid.
			 *
			 * @param mass   Total mass in kg.
			 * @param width  Dimension along X-axis in meters.
			 * @param height Dimension along Y-axis in meters.
			 * @param depth  Dimension along Z-axis in meters.
			 */
			public CuboidSolid(double mass, double width, double height, double depth) {
				m_mass = mass;
				m_width = width;
				m_height = height;
				m_depth = depth;
			}

			@Override
			public double inertiaX() {
				return (1.0 / 12.0) * m_mass * (m_height * m_height + m_depth * m_depth);
			}

			@Override
			public double inertiaY() {
				return (1.0 / 12.0) * m_mass * (m_width * m_width + m_height * m_height);
			}

			@Override
			public double inertiaZ() {
				return (1.0 / 12.0) * m_mass * (m_width * m_width + m_depth * m_depth);
			}
		}

		/**
		 * Thin-walled hollow cuboid. Axis orientation: X-width, Y-depth, Z-height.
		 */
		public static class CuboidHollowThin extends ProjectileShape {
			private final double m_mass;
			private final double m_width;
			private final double m_height;
			private final double m_depth;
			private final double m_thickness;

			/**
			 * Constructs a new CuboidHollowThin.
			 *
			 * @param mass      Total mass in kg.
			 * @param width     Outer dimension along X-axis in meters.
			 * @param height    Outer dimension along Y-axis in meters.
			 * @param depth     Outer dimension along Z-axis in meters.
			 * @param thickness Uniform wall thickness in meters.
			 */
			public CuboidHollowThin(double mass, double width, double height, double depth, double thickness) {
				m_mass = mass;
				m_width = width;
				m_height = height;
				m_depth = depth;
				m_thickness = thickness;
			}

			private double solidInertia(double dim1, double dim2) {
				return (1.0 / 12.0) * m_mass * (dim1 * dim1 + dim2 * dim2);
			}

			@Override
			public double inertiaX() {
				double innerHeight = m_height - 2 * m_thickness;
				double innerDepth = m_depth - 2 * m_thickness;

				double outer = solidInertia(m_height, m_depth);
				double inner = solidInertia(innerHeight, innerDepth);
				return outer - inner;
			}

			@Override
			public double inertiaY() {
				double innerWidth = m_width - 2 * m_thickness;
				double innerHeight = m_height - 2 * m_thickness;

				double outer = solidInertia(m_width, m_height);
				double inner = solidInertia(innerWidth, innerHeight);
				return outer - inner;
			}

			@Override
			public double inertiaZ() {
				double innerWidth = m_width - 2 * m_thickness;
				double innerDepth = m_depth - 2 * m_thickness;

				double outer = solidInertia(m_width, m_depth);
				double inner = solidInertia(innerWidth, innerDepth);
				return outer - inner;
			}
		}

		/**
		 * Ring (thin circular loop). Axis orientation: Z-axis normal to ring plane. (The 2024 Game
		 * Piece - Note)
		 */
		public static class Ring extends ProjectileShape {
			private final double m_mass;
			private final double m_innerRadius;
			private final double m_outerRadius;

			/**
			 * Constructs a new Ring. <b>(The 2024 Game Piece)</b>
			 *
			 * @param mass        Total mass in kg.
			 * @param innerRadius The inner radius in meters.
			 * @param outerRadius The outer radius in meters.
			 */
			public Ring(double mass, double innerRadius, double outerRadius) {
				m_mass = mass;
				m_innerRadius = innerRadius;
				m_outerRadius = outerRadius;

			}

			@Override
			public double inertiaX() {
				double c = (m_outerRadius - m_innerRadius) / 2;
				return (1.0 / 8.0) * m_mass * (5 * c * c + 4 * m_innerRadius + m_innerRadius);
			}

			@Override
			public double inertiaY() {
				return inertiaX();
			}

			@Override
			public double inertiaZ() {
				double c = (m_outerRadius - m_innerRadius) / 2;
				return 1.0 / 4.0 * m_mass * (4 * m_innerRadius * m_innerRadius + 3 * c * c);
			}
		}
	}
}
