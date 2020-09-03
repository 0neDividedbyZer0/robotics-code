package geometry;

import util.Util;

public class RigidTransform implements Interpolable<RigidTransform> {
	private static final RigidTransform identity = new RigidTransform();
	private static final double epsilon = 1E-9;
	
	protected Rotation rotation;
	protected Translation translation;
	
	
	
	public RigidTransform(double x, double y, double theta) {
		rotation = new Rotation(theta);
		translation = new Translation(x, y);
	}
	
	public RigidTransform() {
		rotation = new Rotation();
		translation = new Translation();
	}
	
	public RigidTransform(RigidTransform other) {
		this(other.translation.x(), other.translation.y(), other.rotation.angle());
	}
	
	public RigidTransform(Rotation rotation, Translation translation) {
		this.rotation = rotation;
		this.translation = translation;
	}
	
	public static RigidTransform fromRotation(Rotation rotation) {
		return new RigidTransform(rotation, new Translation());
	}
	
	public static RigidTransform fromTranslation(Translation translation) {
		return new RigidTransform(new Rotation(), translation);
	}
	
	public static final RigidTransform identity() {
		return identity;
	}
	
	public static RigidTransform exp(Twist twist) {
		double angle = twist.dtheta;
		Rotation expTheta = new Rotation(angle); 
		double s, c;
		if(angle < epsilon) {
			s = 1. - angle * angle / 6.0;
			c = angle * 0.5;
		} else {
			s = Math.sin(angle) / angle;
			c = (1.0 - Math.cos(angle)) / angle;
		}
		Translation Vu = new Translation(s * twist.dx - c * twist.dy, c * twist.dx + s * twist.dy);
		return new RigidTransform(expTheta, Vu);
	}
	
	public static Twist log(RigidTransform transform) {
		double dtheta = Math.atan2(transform.rotation.sin, transform.rotation.cos);
		double s, c;
		double x = transform.translation.x;
		double y = transform.translation.y;
		if(dtheta < epsilon) {
			s = 1. - dtheta * dtheta / 6.0;
			c = dtheta * 0.5;
		} else {
			s = Math.sin(dtheta) / dtheta;
			c = (1.0 - Math.cos(dtheta)) / dtheta;
		}
		double den = s * s + c * c;
		double dx = (s * x + c * y) / den;
		double dy = (-c * x + s * y) / den;
		return new Twist(dx, dy, dtheta);
	}
	
	public Translation getTranslation() {
		return translation;
	}
	
	public void setTranslation(Translation newTranslation) {
		translation = newTranslation;
	}
	
	public void setRotation(Rotation newRotation) {
		rotation = newRotation;
	}
	
	public Rotation getRotation() {
		return rotation;
	}
	
	public RigidTransform inverse() {
		return new RigidTransform(rotation.inverse(), translation.rotateBy(rotation.inverse()).inverse());
	}
	
	public RigidTransform transformBy(RigidTransform other) {
		return new RigidTransform(rotation.rotateBy(other.rotation),
				translation.translateBy(other.translation.rotateBy(rotation)));
	}
	
	public RigidTransform normal() {
        return new RigidTransform(rotation.normal(), translation);
    }
	
	/**
     * Finds the point where the heading of this transform intersects the heading of another. Returns (+INF, +INF) if
     * parallel.
     */
    public Translation intersection(RigidTransform other) {
        final Rotation other_rotation = other.getRotation();
        if (rotation.isParallel(other_rotation)) {
            // Lines are parallel.
            return new Translation(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }
        if (Math.abs(rotation.cos()) < Math.abs(other_rotation.cos())) {
            return intersectionInternal(this, other);
        } else {
            return intersectionInternal(other, this);
        }
    }
	
	private static Translation intersectionInternal(RigidTransform a, RigidTransform b) {
        final Rotation a_r = a.getRotation();
        final Rotation b_r = b.getRotation();
        final Translation a_t = a.getTranslation();
        final Translation b_t = b.getTranslation();

        final double tan_b = b_r.tan();
        final double t = ((a_t.x() - b_t.x()) * tan_b + b_t.y() - a_t.y())
                / (a_r.sin() - a_r.cos() * tan_b);
        return a_t.translateBy(a_r.toTranslation().scale(t));
    }
	
	 /**
     * Return true if the heading of this transform is colinear with the heading of another.
     */
    public boolean isColinear(RigidTransform other) {
        final Twist twist = log(inverse().transformBy(other));
        return (Util.epsilonEquals(twist.dy, 0.0, Util.kEpsilon) && Util.epsilonEquals(twist.dtheta, 0.0, Util.kEpsilon));
    }
	
	@Override
	public String toString() {
       return "T:" + translation.toString() + ", R:" + rotation.toString();
	}

	/**
     * Do twist interpolation of this transform assuming constant curvature.
     */
    @Override
    public RigidTransform interpolate(RigidTransform other, double x) {
        if (x <= 0) {
            return new RigidTransform(this);
        } else if (x >= 1) {
            return new RigidTransform(other);
        }
        final Twist twist = RigidTransform.log(inverse().transformBy(other));
        return transformBy(RigidTransform.exp(twist.scale(x)));
    }


}
