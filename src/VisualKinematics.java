import java.awt.Point;

import lejos.utility.Matrix;

public class VisualKinematics {

	/**
	 * Computes the quasi-newton update step using the
	 * Jacobian and the error term.
	 * This is the inverse kinematics to translate 
	 * from errors in the image (delta y) to an update in the
	 * robot movements (delta q)
	 * @return
	 */
	static double[] updateStep(Matrix J, double[] error) {
		return J.solve(new Matrix(error, error.length)).times(0.2).getColumnPackedCopy();
	}
	

	/**
	 * Returns the updated Jacobian using the broyden update using
	 * the error parameters calculated from the newton step
	 * @param J
	 * @param dx
	 * @param dy
	 * @return
	 */
	static Matrix broydenUpdate(Matrix J, double[] dx, double dy[]) {
		Matrix X = new Matrix(dx, dx.length);
		Matrix Y = new Matrix(dy, dy.length);
		
		// J + (Y - J*X)*X' * (1 / X'*X)
		return J.plus( ((Y.minus(J.times(X))).times(X.transpose()).times(1/(X.transpose().times(X).get(0, 0)))).times(0.25));
	}

	/**
	 * Given two points, returns a list of target points for the robot to move
	 * towards along a line.
	 * 
	 * @param start
	 * @param end
	 * @param resolution
	 *            the number of intermediate points to generate
	 * @return
	 */
	public static Point[] createLinePath(double[] start, double[] end, double step_size) {
		double len = 0d;
		for (int i = 0; i < start.length; i++){
			len += (start[i] - end[i])*(start[i] - end[i]);
		}
		len = Math.sqrt(len);
		
		int resolution = (int) (len / step_size);

		Point[] points = new Point[resolution + 1];

		double dx = step_size / len * (end[0] - start[0]);
		double dy = step_size / len * (end[1] - start[1]);

		for (int i = 1; i <= resolution; i++) {
			points[i-1] = new Point();
			points[i-1].setLocation(start[0] + dx * i, start[1] + dy * i);
		}
		points[resolution] = new Point();
		points[resolution].setLocation(end[0], end[1]);

		return points;
	}
}
