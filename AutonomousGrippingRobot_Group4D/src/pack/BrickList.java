package pack;

import java.awt.Color;
import java.util.ArrayList;

import lejos.robotics.localization.PoseProvider;
import lejos.utility.Delay;

public class BrickList {
	
	static ArrayList<Brick> brickList;
	static int zeroPriorities;

	
	public BrickList(){
		brickList=new ArrayList<Brick>();
		zeroPriorities = 0;

	}
	
	public void addBegin(int index,Brick brick){
		brickList.add(index, brick );
		zeroPriorities ++;
		
	}
	public void add(int index,Brick brick){
		brickList.add(index, brick );
		

	}
	public Brick get(int i){
		return brickList.get(i);
	}
	
	public void set(int i, Brick brick){
		 brickList.set(i, brick);
	}
	
	public void remove(int index){
		brickList.remove(index);
	}
	public int size(){
		return brickList.size();
	}
	public void sortPriority (){
		/* this method sorts the list on color priority */
		boolean found = false;
		for (int i=1; i<brickList.size(); i++){
			/* the color priority of the first brick is compared with the color priority of all other bricks inside the list. If a brick is encountered 
			 * with the same or a higher priority, the first brick is put at this position in the list. 
			 */
			if(brickList.get(0).colorPriority <= brickList.get(i).colorPriority){
				add(i, brickList.get(0));
				brickList.remove(0);
				found = true;
				zeroPriorities--;
				break;
			}
		
		}
		if (!found){
			/* if no brick with the same or a higher color priority is found within the list, the brick at the first position is transferred to the back 
			 * of the list.
			 */
			brickList.add(brickList.get(0));
			brickList.remove(0);
			zeroPriorities--;
		}
	}
	public static void sortAtDistance(PoseProvider poseprovider){
		/* this method sorts the bricks in the list with the same color priority as the first birck on the distance between the robot and the brick. */
		
		int priority = brickList.get(0).colorPriority;
		int priorityi = 0;
		
		for (int i=0; i < brickList.size(); i++){
			/* first the amount of bricks with a color priority equal to the first brick are determined. */
			if (brickList.get(i).colorPriority == priority) {
				priorityi = priorityi+1;
			}

		}
		/* for each brick, the distance from the brick to the robot is compared with the distance of the next brick in the list, with the same  
		 * color priority to the robot. If the distance is larger, the two bricks switch positions. This is repeated as many times as there are
		 * bricks with the same color priority as the first brick. After that the list is sorted on distance.
		 */
		for (int i=0; i < priorityi; i++){

			for  (int j=0; j < priorityi-1; j++){
				
				if (poseprovider.getPose().distanceTo(brickList.get(j).waypoint) > poseprovider.getPose().distanceTo(brickList.get(j+1).waypoint)) {
					Brick brick = brickList.get(j+1);
					brickList.set(j+1, brickList.get(j));
					brickList.set(j,brick);
				}
			}
		}

	}
}
