import java.awt.Point;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;
import lejos.utility.Matrix;

public class RobotController {
    private static RegulatedMotor m_motorA;
	private static RegulatedMotor m_motorB;
	private static RegulatedMotor m_motorC;
	private static EV3TouchSensor m_sensor1;

  	private static final int motor_speed = 20;
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
		System.out.format("BEGIN at: (%.2f, %.2f)\n", start[0], start[1]);
		System.out.format("BEGIN with Final Target: (%.2f, %.2f)\n", target[0], target[1]);
		
		Point[] path = VisualKinematics.createLinePath(start, target, 10);
		Matrix J = estimateJacobian();

		Matrix stop = new Matrix(2, 2, 1.0);
		for (Point p : path) {
			J = moveToTarget(J, new double[] {p.getX(), p.getY()});
		}
		
		// Make sure we're on the target
		System.out.format("FINAL at: (%.2f, %.2f)\n", start[0], start[1]);
		target = VisualMain.getTargetPosition();
		System.out.format("FINAL with Final Target: (%.2f, %.2f)\n", target[0], target[1]);
		moveToTarget(J, VisualMain.getTargetPosition());

	}
	
	private static Matrix moveToTarget(Matrix J, double[] target) {
		final double threshold = 10;	// Distance to consider the current position "solved"
		final double normThreshold = 4.5; // Threshold for recalculation of Jacobian
		final double noiseScale = 0.001; // random noise to add to the Jacobian to kick out of loops
		
//		[epos, Bk] = evalRobot2D(l, theta);
		double[] features = VisualMain.getTrackerPosition();
		double[] start = features.clone();
		double[] pfeatures;
		double[] theta = getJointAngles();
		
		System.out.format("START: (%.2f, %.2f)\n", start[0], start[1]);
		System.out.format("TARGET: (%.2f, %.2f)\n", target[0], target[1]);
		
		// Loop until within radius _threshold_
		int iteration = 0;
		while (Math.sqrt(sumsq(arrayDiff(target, features))) > threshold) {
			
			// quit if we're closer to the final target than the current one
			// but not far off from our target
			if (Math.sqrt(sumsq(arrayDiff(VisualMain.getTargetPosition(), features))) > Math.sqrt(sumsq(arrayDiff(target, features)))) {
				System.out.println("Found better solution! Skipping mini target");
				break;
			}
			
//			double left = Math.sqrt(sumsq(new double[] {target[0]-features[0], target[1]-features[1]}));
//			System.out.format("features: (%.2f, %.2f), goal: (%.2f, %.2f) left to go this step: %f.2 \n", features[0], features[1], target[0], target[1], left);
//	        %% Newton-like step
//	        sk = Bk\-(epos-pos);
//	        theta = mod(theta + sk, 2*pi);
			
			double[] error = arrayDiff(target, features);
			
			double[] deltaQ = VisualKinematics.updateStep(J, error);
			System.out.format("update step: [%.2f, %.2fs] degrees?\n", deltaQ[0], deltaQ[1]);
			for (int i = 0; i < theta.length; i++) {
				// clamp movement amount
				deltaQ[i] = Math.min(Math.max(deltaQ[i], -MAX_MOVE), MAX_MOVE);
				theta[i] = (theta[i] + deltaQ[i]) % 360d;
			}
			System.out.format("real step: [%.2f, %.2fs] degrees\n", deltaQ[0], deltaQ[1]);

//	        %% Compute new function values
//	        ppos = epos;
//	        epos = evalRobot2D(l, theta);
//	        %yk = (epos-pos) - (ppos-pos);
//	        yk = epos - ppos;
			
			pfeatures = features.clone();
			rotate(deltaQ);
//			Delay.msDelay(100);	
			features = VisualMain.getTrackerPosition();

			// get deltaY after moving
			double[] deltaY = arrayDiff(features, pfeatures);
			
			// Get the new error after moving
			double[] error2 = arrayDiff(target, features);
			
			//ADDED THIS	
			//check if at target already before moving again
			if ( Math.sqrt(sumsq(error2)) < threshold ){
				System.out.format("AT MINI TARGET! features: [%.2f, %.2f] ?\n target: [%.2f, %.2f]", features[0], features[1], target[0], target[1]);
				break;
			}
			
//	        Update Jacobian (if error increased)
			//TODO:(maybe) SHOULD PATH BE UPDATED AT THIS POINT? BECAUSE WE VEERED?
			
//	        Bk = Bk + ((yk - Bk*sk)*sk')/(sk'*sk);
			if (sumsq(error) < sumsq(error2)) {
				J = VisualKinematics.broydenUpdate(J, deltaQ, deltaY);
			}
			
			// If the jacobian is horrible and useless, we have to
			// reestimate. No other choice.
			if (J.normF() > normThreshold) {
				System.out.println("LOST! Need to reestimate Jacobian.");
				J = estimateJacobian();
			} else {
				System.out.println("Fro. Norm: " + J.normF());
			}
			
			// If we loop a lot, add some random noise to kick
			// it out of a loop
			if (++iteration % 100 == 0) {
				Matrix R = Matrix.random(J.getRowDimension(), J.getColumnDimension());
				J = J.plus(R.times(noiseScale));
			}
			
			// If it iterates for too long, who cares?
			if (iteration >= 500) {
				System.out.println("GAVE UP");
				break;
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
	
	static double[] arrayDiff(double[] x, double[] y) {
		double[] diffs = new double[x.length];
		for (int i = 0; i < x.length; i++)
			diffs[i] = x[i] - y[i];
		return diffs;
	}

	static void rotateTo(double[] angles) {
		getMotorA().rotateTo((int) Math.round(angles[0]), true);	
		getMotorB().rotateTo((int) Math.round(angles[1]), false);
	}
	
	static void rotate(double[] delta_angles) {
		getMotorA().rotate((int) Math.round(delta_angles[0]), true);	
		getMotorB().rotate((int) Math.round(delta_angles[1]), false);
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

	public static RegulatedMotor getMotorC() {
		if (m_motorC == null)
			m_motorC = new EV3LargeRegulatedMotor(MotorPort.C);
			m_motorC.setSpeed(100);
//			m_motorC.setStallThreshold(motor_safe_stall, motor_safe_time);
		return m_motorC;
	}
	
	private static EV3TouchSensor getGripSensor() {
		if (m_sensor1 == null)
			m_sensor1 = new EV3TouchSensor(SensorPort.S1);
		return m_sensor1;
	}
	
	static void grabIt(){
		m_motorC = getMotorC();
		m_motorC.backward();
		
		final int max_turns = 2800;
		final int start_tachos = m_motorC.getTachoCount();

		// keep closing the jaws until either a button is pressed
		// or it moves to the clamping threshold
		// (the stalling detection isn't implemented very well in lejos)
		do {
			if (Button.waitForAnyPress(250) != 0)
				break;
		} while (Math.abs(start_tachos - start_tachos) < max_turns);
		m_motorC.stop(true);
		
		//ATTEMPT AT ERROR CONTROL FOR BREAKING BUT DOESN'T SEEM TO WORK
		//while (m_motorC.isMoving()){
			//if (m_motorC.isStalled() == true) m_motorC.stop();
		//}
		//carrying = true;
	}
	
	static void letItGo(){
		
		getMotorC().forward();
		
		float[] sample = new float[1];
		do {
			// If button press, stop.
			if (Button.waitForAnyPress(100) != 0)
				break;
			// Check the touch sensor
			getGripSensor().fetchSample(sample, 0);
		} while (sample[0] < 0.5);
		getMotorC().flt();
		
		//ATTEMPT AT ERROR CONTROL FOR BREAKING BUT DOESN'T SEEM TO WORK
		//while (m_motorC.isMoving()){
			//if (m_motorC.isStalled() == true) m_motorC.stop();
		//}
		//m_motorC.stop();
		//carrying = false;
	}
		
}
