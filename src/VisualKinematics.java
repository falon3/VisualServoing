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
	double[] updateStep(Matrix J, double[] error) {
		return J.solve(new Matrix(error, error.length)).getColumnPackedCopy();
	}
	

	/**
	 * Returns the updated Jacobian using the broyden update using
	 * the error parameters calculated from the newton step
	 * @param J
	 * @param dx
	 * @param dy
	 * @return
	 */
	Matrix broydenUpdate(Matrix J, double[] dx, double dy[]) {
		Matrix X = new Matrix(dx, dx.length);
		Matrix Y = new Matrix(dy, dy.length);
		
		// J + (Y - J*X)*X' * (1 / X'*X)
		return J.plus( (Y.minus(J.times(X))).times(X.transpose()).times(1/X.transpose().times(X).get(0, 0)));
	}

}
