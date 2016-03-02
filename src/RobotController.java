import lejos.utility.Matrix;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;

public class RobotController {
    private static RegulatedMotor m_motorA;
	private static RegulatedMotor m_motorB;

  	private static final int motor_speed = 70;
	private static final int motor_accel = 30;
	
	private static final double[] gear_ratios = {15d/30d, 1d, -1d};

	/**
	 * Moves the joints slightly to find an estimate for
	 * the jacobian at the current location.
	 * @return
	 */
	public static Matrix estimateJacobian() {
        Matrix J = new Matrix(2,2);
        double deltaTheta = 0.0; // wiggle this many degrees to get initial J
        RegulatedMotor motorA=getMotorA();
        RegulatedMotor motorB=getMotorB();
        RegulatedMotor[] motors= new RegulatedMotor[] {motorA,motorB};
        
        for (int j=0; j<2; j++){
            double[] q0 = VisualMain.getTrackerPosition();
            motors[j].rotateTo((int)Math.round(deltaTheta));
            // may need a delay in here??
            Delay.msDelay(100);
            double[] q1 = VisualMain.getTrackerPosition();
            J.set(0, j, q1[0]-q0[0]/deltaTheta);
            J.set(1, j, q1[1]-q1[1]/deltaTheta);
            //move joints back after
            motors[j].rotateTo(-(int)Math.round(deltaTheta));
        }
     	return J;
	}
	
	/**
	 * Moves the tracked object to the target in the tracker
	 */
	static void moveToTarget() {
		 Matrix J = estimateJacobian();
		 double eps = 0.0001;
		 
		 double[] features = VisualMain.getTrackerPosition();;
		 double[] fprev;
		 double[] target = VisualMain.getTargetPosition();
		 
		 double[] error = new double[target.length];

		 double[] fstart = features.clone();
		 double[] q = getJointAngles();;
		 double[] dq = new double[q.length];
		 
		 double distance = 0d;
		 
		 //====
		 // Run the first step
		 J = estimateJacobian();
		 
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

	static void rotateTo(double[] angles) {
		getMotorA().rotateTo((int) Math.round(angles[0]));
		getMotorB().rotateTo((int) Math.round(angles[1]));
	}
	
	static double[] getJointAngles() {
		return new double[] {getMotorA().getTachoCount(), getMotorB().getTachoCount()};
	}
	
    /**
	 * Gets motorA as a singleton. Prevents the port from being opened twice.
	 */
	private static RegulatedMotor getMotorA() {
		if (m_motorA == null)
			m_motorA = new EV3LargeRegulatedMotor(MotorPort.A);
		return m_motorA;
	}
	/**
	 * Gets motorB as a singleton. Prevents the port from being opened twice.
	 */
	private static RegulatedMotor getMotorB() {
		if (m_motorB == null)
			m_motorB = new EV3LargeRegulatedMotor(MotorPort.B);
		return m_motorB;
	}
}
