package geometry;
import java.text.DecimalFormat;

public class Twist {
	public double dx, dy, dtheta;
	
	protected static final Twist identity = new Twist(0.,0.,0.);
	
	//dtheta is in radians!!!
	public Twist(double dx, double dy, double dtheta) {
		this.dx = dx;
		this.dy = dy;
		this.dtheta = dtheta;
	}
	
	public static final Twist identity() {
		return identity;
	}
	
	public double getDegrees() {
		return Math.toDegrees(dtheta);
	}
	
	public Twist scale(double s) {
		return new Twist(dx * s, dy * s, dtheta * s);
	}
	
	@Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return "(" + fmt.format(dx) + "," + fmt.format(dy) + "," + fmt.format(getDegrees()) + " deg)";
    }
}
