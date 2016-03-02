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
		// return pinv(J) * error
		return null;
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
		// Jk+1 = Jk + (dy - J dx) dx^T / ()
		return null;
	}

}
