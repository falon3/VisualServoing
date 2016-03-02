
public class VisualMain {
	public static TrackerReader tracker;
	public static void main(String[] args) {
		// TODO Auto-generated method stub
	
        tracker = new TrackerReader();
        tracker.start();
        while (true) {
            //Delay.msDelay(1000);
	    try {
		Thread.sleep(1000);         //1000 milliseconds is one second.
	    } catch(InterruptedException ex) {
		Thread.currentThread().interrupt();
	    }
            System.out.println(tracker.x + " " + tracker.y);
        }

	}
	
	/**
	 * Get the tracker information
	 * @return
	 */
	double[] getTrackerPosition() {
		return new double[] {tracker.x, tracker.y};
	}
	
	/**
	 * Get the tracker target information
	 * @return
	 */
	double[] getTargetPosition() {
		return new double[] {tracker.targetx, tracker.targety};
	}

}
