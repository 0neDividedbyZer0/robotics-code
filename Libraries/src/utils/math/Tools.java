package utils.math;

public class Tools {
	
	private Tools() {
	}
	
	public static double d2r(double deg) {
		return (Math.PI / 180.0) * deg;
	}
	
	public static double r2d(double rad) {
		return (180.0 / Math.PI) * rad;
	}
	
	public static double boundRadians(double rad) {
		double angle;
		if(rad >= 0) {
			angle = rad % (2 * Math.PI);
		} else {
			angle = (2 * Math.PI) + (-rad % (2 * Math.PI));
		}
		
		return angle;
	}

}
