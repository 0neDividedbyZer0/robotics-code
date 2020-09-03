package util;

import java.util.ArrayList;
import java.util.List;

import trajectory_lib.Coord;

public class LinearRegression {
	private List<Coord> regressionPoints;
	
	private double sum_of_x_n, num_points, sum_x_n_2, sum_y_n, sum_x_n_y_n;
	
	public double a,b;
	public LinearRegression(List<Coord> points) {
		regressionPoints = new ArrayList<Coord>();
		for(Coord point : points) {
			regressionPoints.add(point);
		}
		sum_of_x_n = 0.0;
		sum_x_n_2 = 0.0;
		sum_y_n = 0.0;
		sum_x_n_y_n = 0.0;
		num_points = regressionPoints.size();
		for(int i = 0; i < num_points; i++) {
			double x_n = regressionPoints.get(i).x;
			double y_n = regressionPoints.get(i).y;
			sum_of_x_n += x_n;
			sum_x_n_2 += x_n * x_n;
			sum_y_n += y_n;
			sum_x_n_y_n += x_n * y_n;
		}
		double det = sum_of_x_n * sum_of_x_n - num_points * sum_x_n_2;
		a = (sum_of_x_n * sum_y_n - num_points * sum_x_n_y_n) / det;
		b = (-sum_x_n_2 * sum_y_n + sum_of_x_n * sum_x_n_y_n) / det;
	}
	


}
