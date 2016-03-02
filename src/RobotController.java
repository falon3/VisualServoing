import lejos.utility.Matrix;

public class RobotController {
	/**
	 * Moves the joints slightly to find an estimate for
	 * the jacobian at the current location.
	 * @return
	 */
	static Matrix estimateJacobian() {
		// for each joint
		//	   y0 = getTrackerPosition()
		//     move joint slightly
		//	   y1 = getTrackerPosition()
		//	   insert y1 - y0 into the jacobian as a column vector
		// return J
		
		return null;
	}
	
	/**
	 * Moves the tracked object to the target in the tracker
	 */
	static void moveToTarget() {
		 Matrix J = estimateJacobian();
		 double eps = 0.0001;
		 
		 double[] features;
		 double[] fprev;
		 double[] target = VisualMain.getTargetPosition();
		 
		 double[] error = new double[target.length];

		 double[] fstart = features.clone();
		 double[] q;
		 double[] dq = new double[q.length];
		 
		 double distance = 0d;
		 
		 //====
		 // Run the first step
		 J = estimateJacobian();

		 // Get the features
		 features = VisualMain.getTrackerPosition();
		 
		 // Compute the error
		 for (int i = 0; i < features.length; i++) {
			 error[i] = features[i] - target[i];
		 }

		 // Compute the update step
		 dq = VisualKinematics.updateStep(J, error);
		 
		 // Update the joint angles
		 for (int i = 0; i < q.length; i++) {
			 q[i] = q[i] + dq[i];
		 }
		 //--
		 
		 // Run the Broyden's method loop
		 /* (pre: x_0, f_0 has been computed.)
		  * find f_n
		  * dx_n = x_n - x_{n-1}
		  * df_n = f_n - f_{n-1}
		  * Update J using dx_n, df_n
		  * x_{n+1} = x_n + inv(J_n) * f_n
		  * Loop
		  */
		 do {
			 // Get features
			 fprev = features.clone();
			 features = VisualMain.getTrackerPosition();
			 double[] dy = new double[features.length];
			 Matrix A;
			 
			 // Compute the error
			 distance = 0d;
			 for (int i = 0; i < features.length; i++) {
				 error[i] = features[i] - target[i];
				 dy[i] = features[i] - fprev[i];
				 distance += (features[i] - fstart[i])*(features[i] - fstart[i]);
			 }
			 
			 // If error is small enough, stop
			 if (sumsq(error) < eps) {
				 break;
			 }
			 
			 // If the euclidean distance is large, recompute the Jacobian
			 if (Math.sqrt(distance) > 10) {
				 J = estimateJacobian();
				 fstart = features;
			 }
			 
			 // Broyden step update the Jacobian
			 // dq = q(n+1) - q(n) = angle update (Should this be the real angles? or compute by updatestep?)
			 // dy = f(n+1) - f(n) = change in the loop iteration
			 J = VisualKinematics.broydenUpdate(J, dq, dy);
			 
			 // Compute the update step
			 // Should this be features or error?
			 dq = VisualKinematics.updateStep(J, features);
			 
			 // Update the joint angles
			 for (int i = 0; i < q.length; i++) {
				 q[i] = q[i] + dq[i];
			 }
			 
			 rotateTo(q);
			 
		 } while (true);
		 
		 //====
	}
	
	/**
	 * Computes the squared sum of a list X.
	 * @param X
	 * @return
	 */
	static double sumsq(double[] X) {
		double s = 0d;
		for (double x : X)
			s += x*x;
		return s;
	}
}
