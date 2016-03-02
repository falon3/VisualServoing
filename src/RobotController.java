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
        double deltaTheta = 0.0 // wiggle this many degrees to get initial J
        RegulatedMotor motorA=GetMotorA();
        RegulatedMotor motorB=GetMotorB();
        RegulatedMotor motors= new RegulatedMotor[] {motorA,MotorB};
        
        for (int j=0; j<2; j++){
            double q0 = VisualMain.getTrackerPosition();
            motors[j].rotateTo(deltaTheta);
            // may need a delay in here??
            Delay.msdelay(100)
            double q1 = VisualMain.getTrackerPosition();
            J.set(0, j, q1.x-q0.x/deltaTheta);
            J.set(1, j, q1.y-q1.y/deltaTheta);
            //move joints back after
            motors[j].rotateTo(-deltaTheta);
        }
     	return J;
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
