package geometry;


import java.text.DecimalFormat;

//Holds information about translations in the plane
public class Translation implements Interpolable<Translation> {
	protected static final Translation identity = new Translation();
	
	protected double x,y;
	
	public static final Translation identity() {
		return identity;
	}
	
	public Translation(double x, double y) {
		this.x = x;
		this.y = y;
	}
	
	public Translation() {
		x = 0.;
		y = 0.;
	}
	
	public Translation(Translation other) {
		x = other.x;
		y = other.y;
	}
	
	public Translation(Translation start, Translation end) {
		x = end.x - start.x;
		y = end.y - start.y;
	}
	
	public double norm() {
		return Math.hypot(x, y);
	}
	
	public double norm2() {
		return x * x + y * y;
	}
	
	public static double dot(Translation a, Translation b) {
		return a.x * b.x + a.y * b.y;
	}
	
	public double x() {
		return x;
	}
	
	public double y() {
		return y;
	}
	
	public Translation translateBy(Translation other) {
		return new Translation(x + other.x, y + other.y);
	}
	
	public Translation rotateBy(Rotation rotation) {
		double t_x = rotation.cos() * x - rotation.sin() * y;
		double t_y = rotation.sin() * x + rotation.cos() * y;
		return new Translation(t_x, t_y);
	}
	
	public Translation inverse() {
		return new Translation(-x,-y);
	}
	
	public static Rotation getAngle(Translation a, Translation b) {
		double cos = dot(a,b)/(a.norm() * b.norm());
		if(Double.isNaN(cos)) {
			return new Rotation();
		}
		return new Rotation(Math.acos(cos));
	}
	
	public static double cross(Translation a, Translation b) {
		return a.x * b.y - a.y * b.x;
	}
	
	public Translation scale(double s) {
		return new Translation(x * s, y * s);
	}
	
	public Rotation direction() {
        return new Rotation(x, y, true);
    }
	
	@Override
	public Translation interpolate(Translation other, double t) {
		if(t <= 0.0) {
			return new Translation(this);
		} else if(t >= 1.0) {
			return new Translation(other);
		} else {
			return extrapolate(other, t);
		}
	}
	
	public Translation extrapolate(Translation other, double t) {
        return new Translation(t * (other.x - x) + x, t * (other.y - y) + y);
    }
	
	@Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return "(" + fmt.format(x) + "," + fmt.format(y) + ")";
    }
}
