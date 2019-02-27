import lejos.robotics.navigation.MovePilot;

public class Driving {
	MovePilot pilot;
	
	public Driving(MovePilot p) {
		pilot = p;
	}
	public void travelAndRotate( ) {
		pilot.travel(20);
		pilot.rotate(180);
	}
}
