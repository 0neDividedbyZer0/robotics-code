package util;
import java.util.ArrayList;
import java.util.List;

import org.ojalgo.matrix.PrimitiveMatrix;

import jaci.pathfinder.Waypoint;



/**
 * Contains basic functions that are used often.
 */
public class Util {
	
	public static final double kEpsilon = 1E-6;
    /** Prevent this class from being instantiated. */
    private Util() {
    }

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }
    
    /**
     * Make sure the dimensions are correct
     * @param input
     * @param min
     * @param max
     * @return Capped matrix componentwise
     */
    public static PrimitiveMatrix cap(PrimitiveMatrix input, PrimitiveMatrix min, PrimitiveMatrix max) {
    	double[][] arr = new double[(int) input.countRows()][(int) input.countColumns()];
    	for(int i = 0; i < arr.length; i++) {
    		for(int j = 0; j < arr[0].length; j++) {
    			arr[i][j] = limit(input.get(i, j), min.get(i, j), max.get(i, j));
    		}
    	}
    	return PrimitiveMatrix.FACTORY.rows(arr);
    }

    public static String joinStrings(String delim, List<?> strings) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean allCloseTo(List<Double> list, double value, double epsilon) {
        boolean result = true;
        for (Double value_in : list) {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }
    
    public static List<Waypoint> reflectField(List<Waypoint> points) {
    	List<Waypoint> reflected = new ArrayList<Waypoint>();
    	for(int i = 0; i < points.size(); i++) {
    		reflected.add(new Waypoint(points.get(i).x,-points.get(i).y,-points.get(i).angle));
    	}
    	return reflected;
    }
}