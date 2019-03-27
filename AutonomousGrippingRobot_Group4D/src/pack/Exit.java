package pack;

import lejos.hardware.Button;

public class Exit extends Thread{

	public void run(){
		while(true){
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			if(Button.ENTER.isDown()){
				System.exit(0);
			}

		}}

}
