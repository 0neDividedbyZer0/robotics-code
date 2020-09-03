package geometry;


import java.text.DecimalFormat;

import util.Util;


public class Rotation implements Interpolable<Rotation> {
	protected static final Rotation identity = new Rotation();
	
	protected double angle;
	protected double cos, sin;
	
	 public Rotation(double x, double y, boolean normalize) {
	        cos = x;
	        sin = y;
	        if (normalize) {
	            normalize();
	        }
	    }
	
	//Radians!!!
	public Rotation(double angle) {
		this.angle = boundHalfRadians(angle);
		cos = Math.cos(angle);
		sin = Math.sin(angle);
	}
	
	public Rotation(Rotation other) {
		angle = other.angle;
		cos = other.cos;
		sin = other.sin;
	}
	
	public Rotation(Translation direction, boolean normalize) {
        this(direction.x(), direction.y(), normalize);
    }
	
	public Rotation() {
		angle = 0.0;
		cos = 1.0;
		sin = 0.0;
	}
	
	public double angle() {
		return angle;
	}
	
	public double getDegrees() {
		return Math.toDegrees(angle);
	}
	
	public double cos() {
		return cos;
	}
	
	public double sin() {
		return sin;
	}
	
	public void normalize() {
        double magnitude = Math.hypot(cos, sin);
        if (magnitude > Util.kEpsilon) {
            sin /= magnitude;
            cos /= magnitude;
        } else {
            sin = 0;
            cos = 1;
        }
    }
	
	public Rotation(Rotation start, Rotation end) {
		this(boundHalfRadians(end.angle - start.angle));
	}
	
	public Rotation rotateBy(Rotation other) {
		double addedAngle = angle + other.angle;
		return new Rotation(boundHalfRadians(addedAngle));
	}
	
	public Rotation inverse() {
		return new Rotation(-angle);
	}
	
	private static double boundHalfRadians(double angle_radians) {
		double temporaryAngle = angle_radians;
        while (temporaryAngle >= Math.PI) {
        	temporaryAngle -= 2.0 * Math.PI; 
        }
        while (temporaryAngle < -Math.PI) {
        	temporaryAngle += 2.0 * Math.PI;
        }
        return temporaryAngle;
    }
	
	public Rotation normal() {
		return new Rotation(-sin,cos,false);
	}
	
	public Translation toTranslation() {
        return new Translation(cos, sin);
    }
	
	public boolean isParallel(Rotation other) {
        return Util.epsilonEquals(Translation.cross(toTranslation(), other.toTranslation()), 0.0, Util.kEpsilon);
    }
	
	public double tan() {
        if (Math.abs(cos) < Util.kEpsilon) {
            if (sin >= 0.0) {
                return Double.POSITIVE_INFINITY;
            } else {
                return Double.NEGATIVE_INFINITY;
            }
        }
        return sin / cos;
    }

	@Override
    public Rotation interpolate(Rotation other, double x) {
        if (x <= 0) {
            return new Rotation(this);
        } else if (x >= 1) {
            return new Rotation(other);
        }
        double angle_diff = inverse().rotateBy(other).angle;
        return this.rotateBy(new Rotation(angle_diff * x));
    }
	
	@Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return "(" + fmt.format(getDegrees()) + " deg)";
    }
}
