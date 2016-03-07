import java.awt.Point;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;
import lejos.utility.Matrix;

public class RobotController {
    private static RegulatedMotor m_motorA;
	private static RegulatedMotor m_motorB;
	private static RegulatedMotor m_motorC;

	private static final int clamp_tachs = 3160;
  	private static final int motor_speed = 40;
	private static final int motor_accel = 10;
	// dont know how to make these work
	//private static final int motor_safe_stall = 5;
	//private static final int motor_safe_time = 3; //seconds??
	
	private static final double[] gear_ratios = {15d/30d, 1d, -1d};
	
	private static final double MAX_MOVE = 15;

	/**
	 * Moves the joints slightly to find an estimate for
	 * the jacobian at the current location.
	 * @return
	 */
	public static Matrix estimateJacobian() {
        Matrix J = new Matrix(2,2);
        double deltaTheta = 10.0; // wiggle this many degrees to get initial J
        RegulatedMotor motorA=getMotorA();
        RegulatedMotor motorB=getMotorB();
        RegulatedMotor[] motors= new RegulatedMotor[] {motorA,motorB};
        
        for (int j=0; j<2; j++){
            double[] q0 = VisualMain.getTrackerPosition();
            motors[j].rotateTo((int)Math.round(motors[j].getTachoCount() + deltaTheta));
            // may need a delay in here??
            Delay.msDelay(500);
            double[] q1 = VisualMain.getTrackerPosition();
            J.set(0, j, (q1[0]-q0[0])/deltaTheta);
            J.set(1, j, (q1[1]-q0[1])/deltaTheta);
            //move joints back after
            motors[j].rotateTo((int)Math.round(motors[j].getTachoCount() - deltaTheta));
            Delay.msDelay(500);
        }
     	return J;
	}
	
	/**
	 * Moves the tracked object to the target in the tracker
	 */
	static void moveToTarget() {
		double[] target = VisualMain.getTargetPosition();
		double[] start = VisualMain.getTrackerPosition();
		double[] final_target = target;
		System.out.format("BEGIN with Final Target: (%.2f, %.2f)\n", final_target[0], final_target[1]);
		Point[] path = VisualKinematics.createLinePath(start, target, 40);
		Matrix J = estimateJacobian();

		Matrix stop = new Matrix(2, 2, 1.0);
		for (Point p : path) {
			J = moveToTarget(J, new double[] {p.getX(), p.getY()});
		}
		RobotController.grabIt();
	}
	
	private static Matrix moveToTarget(Matrix J, double[] target) {
		double threshold = 5;
//		[epos, Bk] = evalRobot2D(l, theta);
		double[] features = VisualMain.getTrackerPosition();
		double[] pfeatures;
		double[] theta = getJointAngles();

		while( (Math.abs((int)(target[0]-features[0])) > 5) && (Math.abs((int)(target[1]-features[1])) > 5)){
//			double left = Math.sqrt(sumsq(new double[] {target[0]-features[0], target[1]-features[1]}));
//			System.out.format("features: (%.2f, %.2f), goal: (%.2f, %.2f) left to go this step: %f.2 \n", features[0], features[1], target[0], target[1], left);
//	        %% Newton-like step
//	        sk = Bk\-(epos-pos);
//	        theta = mod(theta + sk, 2*pi);
			
			double[] error = new double[features.length];
			for (int i = 0; i < features.length; i++) {
				error[i] = target[i] - features[i];
			}
			
			double[] deltaQ = VisualKinematics.updateStep(J, error);
			System.out.format("update step: [%.2f, %.2fs] degrees?\n", deltaQ[0], deltaQ[1]);
			for (int i = 0; i < theta.length; i++) {
				theta[i] = (theta[i] + Math.min(Math.max(deltaQ[i], -MAX_MOVE), MAX_MOVE)) % 360d;
			}

//	        %% Compute new function values
//	        ppos = epos;
//	        epos = evalRobot2D(l, theta);
//	        %yk = (epos-pos) - (ppos-pos);
//	        yk = epos - ppos;
			
			pfeatures = features.clone();
			rotateTo(theta);
			Delay.msDelay(500);			
			features = VisualMain.getTrackerPosition();
			
			//ADDED THIS	
			//check if at target already before moving again
			if ( (Math.abs((int)(target[0]-features[0])) < 5) && (Math.abs((int)(target[1]-features[1])) < 5) ){
				System.out.format("AT MINI TARGET! features: [%.2f, %.2f] ?\n target: [%.2f, %.2f]", features[0], features[1], target[0], target[1]);
				break;
			}
			
			// get deltaY after moving
			double[] deltaY = new double[features.length];
			for (int i = 0; i < features.length; i++) {
				deltaY[i] = features[i] - pfeatures[i];
			}
			// Get the new error after moving
			double[] error2 = new double[features.length];
			for (int i = 0; i < features.length; i++) {
				error2[i] = target[i] - features[i];
			}
			
			System.out.format("sumsq error: %.2f  error2 = [%.2f, %.2f]",Math.sqrt(sumsq(error2)), error2[0], error2[1]);
//	        if norm(epos - pos) < thresh, break; end;
			if (Math.sqrt(sumsq(error2)) < threshold) {
				System.out.format("error within thresh for this mini Goal! features: [%.2f, %.2f] ?\n target: [%.2f, %.2f]", features[0], features[1], target[0], target[1]);
				break;
			}
	        
//	        Update Jacobian (if error increased)
//	        Bk = Bk + ((yk - Bk*sk)*sk')/(sk'*sk);
			if (sumsq(error) < sumsq(error2)) {
				J = VisualKinematics.broydenUpdate(J, deltaQ, deltaY);
			}
		}
		return J;
	}

	/**
	 * Converts a matrix to a string for convenient debugging and output.
	 * @param M matrix to print
	 * @return string representing the matrix
	 */
	protected static String matrixToString(Matrix M) {
		StringBuilder res = new StringBuilder("[\n");
		for (double[] row : M.getArray()) {
			String sep = "\t";
			for (double col : row) {
				res.append(sep);
				if (col == 0)
					res.append("      0");
				else
					res.append(String.format("%1.5f", col));
				sep = ",\t";
			}
			res.append('\n');
		}
		res.append("]");
		return res.toString();
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
		getMotorA().rotateTo((int) Math.round(angles[0]), true);	
		getMotorB().rotateTo((int) Math.round(angles[1]), false);

	}
	
	static double[] getJointAngles() {
		return new double[] {getMotorA().getTachoCount(), getMotorB().getTachoCount()};//, getMotorC().getTachoCount()};
	}
	
    /**
	 * Gets motorA as a singleton. Prevents the port from being opened twice.
	 */
	private static RegulatedMotor getMotorA() {
		if (m_motorA == null)
			m_motorA = new EV3LargeRegulatedMotor(MotorPort.A);
			m_motorA.setSpeed(motor_speed);
//			m_motorA.setStallThreshold(motor_safe_stall, motor_safe_time);
		return m_motorA;
	}
	/**
	 * Gets motorB as a singleton. Prevents the port from being opened twice.
	 */
	private static RegulatedMotor getMotorB() {
		if (m_motorB == null)
			m_motorB = new EV3LargeRegulatedMotor(MotorPort.B);
			m_motorB.setSpeed(motor_speed);
//			m_motorB.setStallThreshold(motor_safe_stall, motor_safe_time);
		return m_motorB;
	}
	
	private static RegulatedMotor getMotorC() {
		if (m_motorC == null)
			m_motorC = new EV3LargeRegulatedMotor(MotorPort.C);
			m_motorC.setSpeed(100);
//			m_motorC.setStallThreshold(motor_safe_stall, motor_safe_time);
		return m_motorC;
	}
	
	static void grabIt(){
		m_motorC = getMotorC();
		m_motorC.rotate(-clamp_tachs, true);
		//ATTEMPT AT ERROR CONTROL FOR BREAKING BUT DOESN'T SEEM TO WORK
		//while (m_motorC.isMoving()){
			//if (m_motorC.isStalled() == true) m_motorC.stop();
		//}
		//carrying = true;
	}
	
	static void letItGo(){
		m_motorC = getMotorC();
		m_motorC.rotate(clamp_tachs, true);
		//ATTEMPT AT ERROR CONTROL FOR BREAKING BUT DOESN'T SEEM TO WORK
		//while (m_motorC.isMoving()){
			//if (m_motorC.isStalled() == true) m_motorC.stop();
		//}
		//m_motorC.stop();
		//carrying = false;
	}
		
}
