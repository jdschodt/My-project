package Sensors;
import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.Sound;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.utility.Delay;

public class BRICK {
	private static Brick brick;
	private static GraphicsLCD LCD;
	public BRICK() {
		 brick = BrickFinder.getDefault();
		 LCD = brick.getGraphicsLCD();
		 System.out.println("Brick configured");
		 Delay.msDelay(1000);
	}
	public void println(String Text){
		System.out.println(Text);
	}
	public void Clear() {
	    LCD.clear();
	}
	public void beep(int type,int volume) {
		Sound.setVolume(volume);
		switch (type){
		case 1:{
			Sound.beep(); //1 beep
			break;
		}
		case 2:{
			Sound.twoBeeps();//2 sequential beeps
			break;
		}
		
		case 3:{
			Sound.beepSequence();//Downward tones
			break;
		}
		case 4:{
			Sound.beepSequenceUp();//Upward tones
			break;
		}
		case 5:{
			Sound.buzz();
			break;
		}}
		
			
		}

		
		
	}
	
