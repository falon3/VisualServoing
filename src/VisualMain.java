import lejos.hardware.Button;

/** VisualMain handles the call to move robot arm to selected target 
 *  and then grab an item and move it to the next selected position 
 *  before releasing it there, in a loop. using the Visual Tracker tool.
 * */
public class VisualMain {
	public static TrackerReader tracker;
	public static boolean open;
	public static void main(String[] args) {
		// TODO Auto-generated method stub
	
        tracker = new TrackerReader();
        tracker.start();
        // start with robot clamp open
		RobotController.letItGo();
		open = true;
		
	    while (true) {
	    	System.out.format("Select target \nselect tracker \nthen push any button\n");
	        Button.waitForAnyPress();
	        RobotController.moveToTarget();
	        
	        if (open){
	        	System.out.format("press any button to stop...");
	        	RobotController.grabIt();
	        } else {
	        	System.out.format("releasing object...");
	        	RobotController.letItGo();
	        }
	        
	        // Toggle open/close mode
	        open = !open;
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
