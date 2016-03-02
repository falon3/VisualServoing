import lejos.utility.Matrix;

public class RobotController {
	/**
	 * Moves the joints slightly to find an estimate for
	 * the jacobian at the current location.
	 * @return
	 */
	Matrix estimateJacobian() {
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
	void moveToTarget() {
		// J = estimateJacobian
		// while ( error > eps )
		//    if euclidean distance from start to now is too big
		//	      J = estimateJacobian
		//    error = getTrackerError
		//	  dx = VisualKinematics.updateStep(J, error)	// (update x using xdot)
		//	  moveJointAngles(dx)
		//    J = VisualKinematics.broydenUpdate(J, dx, error) // CHECK which dx and error this is
	}
	
}
