import lejos.hardware.Button;

public class VisualMain {
	public static TrackerReader tracker;
	public static boolean open;
	public static void main(String[] args) {
		// TODO Auto-generated method stub
	
        tracker = new TrackerReader();
		RobotController.letItGo();
		open = true;
   
        while (true) {
        	tracker.start();// TODO FIGURE OUT HOW TO RESTART tracker
        	System.out.format("Select target \nselect tracker \nthen push any button\n");
            Button.waitForAnyPress();
            RobotController.moveToTarget();
            if (open==true){
            	System.out.format("press any button to pick up");
            	Button.waitForAnyPress();
            	RobotController.grabIt();
            	open = false;
            	//TODO WHY DOESN'T THIS CONTINUE THE LOOP AGAIN AFTER IT GRABS?
            }
            else{
            	RobotController.letItGo();
            	open = true;
            }
        }

	}
	
	/**
	 * Get the tracker information
	 * @return
	 */
	static double[] getTrackerPosition() {
		return new double[] {tracker.x, tracker.y};
	}
	
	/**
	 * Get the tracker target information
	 * @return
	 */
	static double[] getTargetPosition() {
		return new double[] {tracker.targetx, tracker.targety};
	}

}
