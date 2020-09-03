package motion.trajectory;

import utils.math.Tools;

public class Spline {
	protected double a,b,c,d,e;
	//delta is the knot distance
	protected double x_offset, y_offset, ang_offset, delta;
	
	public enum SplineType {
		CUBIC,
		QUINTIC
	}
	
	public Spline(Waypoint start, Waypoint end, SplineType type) {
		this.x_offset = start.x;
		this.y_offset = start.y;
		this.ang_offset = Tools.boundRadians(Math.atan((end.y - start.y)/(end.x - start.x)));
		this.delta = Math.hypot(end.x - start.x, end.y - start.y);
		
		if(type == SplineType.CUBIC) {
			generateCubic(start, end);
		} else if(type == SplineType.QUINTIC) {
			generateQuintic(start, end);
		}
	}
	
	private void generateCubic(Waypoint start, Waypoint end) {
		
		this.a = 0;
		this.b = 0;
		
		double delta_a0 = Math.tan(Tools.boundRadians(start.heading - ang_offset));
		double delta_a1 = Math.tan(Tools.boundRadians(end.heading - ang_offset));
		
		this.c = (delta_a0 + delta_a1) / (delta * delta);
		this.d = - (2.0 * delta_a0 + delta_a1) / (delta);
		this.e = delta_a0;
	}
	
	private void generateQuintic(Waypoint start, Waypoint end) {
		double delta_a0 = Math.tan(Tools.boundRadians(start.heading - ang_offset));
		double delta_a1 = Math.tan(Tools.boundRadians(end.heading - ang_offset));
		double d = this.delta;
		
		this.a = -(3.0 * (delta_a0 + delta_a1)) / (d * d * d * d);
		this.b = (8.0 * delta_a0 + 7.0 * delta_a1) / (d * d * d);
		this.c = -(6.0 * delta_a0 + 4.0 * delta_a1) / (d * d);
		this.d = 0.0;
		this.e = delta_a0;
	}
	

	

}
