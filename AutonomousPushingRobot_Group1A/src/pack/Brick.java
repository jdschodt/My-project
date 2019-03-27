package pack;
import lejos.robotics.navigation.Waypoint;

public class Brick {
	
	Waypoint waypoint ;
	int colorPriority = 0;
	
	public Brick(Waypoint waypoint) {
		this.waypoint = waypoint;
	    colorPriority = 0;
	}
	
	void changeWaypoint(Waypoint point){
		/* this method changes the waypoint of an existing brick */
		waypoint=point;
	}

	void changeColorPriority(int newPriority){
		/* this method changes the color priority of the brick */
		colorPriority = newPriority;
	}
}
