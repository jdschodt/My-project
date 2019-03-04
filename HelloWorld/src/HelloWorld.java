import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.Font;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.utility.Delay;

import java.util.Random;
import lejos.hardware.motor.Motor;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;

public class HelloWorld
{
    public static void main(String[] args)
    {
        //System.out.println("Running...");
        GraphicsLCD g = BrickFinder.getDefault().getGraphicsLCD();
        final int SW = g.getWidth();
        final int SH = g.getHeight();
        Button.LEDPattern(4);
        Sound.beepSequenceUp();
        g.setFont(Font.getDefaultFont());
        g.drawString("HelloWorld!", SW/2, SH/2, GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
        Button.LEDPattern(3);
        Delay.msDelay(4000);
        g.clear();
        String str;
        for (int i=0;i<=10;i++) {
        	 Button.LEDPattern(i);
        	 str=Integer.toString(i);
             g.drawString(str, SW/2, SH/2, GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
             Sound.beepSequenceUp();
             Delay.msDelay(3000);
             g.clear();
        }
        Sound.beepSequence();
        Delay.msDelay(500);
        Button.LEDPattern(0);
        
        MovePilot pilot;
    	Random ran = new Random();
    	
    	int period = 3; // for waiting
    	int fastTravelSpeed = 8;
    	int slowTravelSpeed = 2;
    	
    	public static void main(String[] args) {
    		new DriveTest();
    	}
    	public DriveTest() {
    		float d = 5.62f; //Diameter of the wheels
    		float s = 30f;
    		float y = 9.8f;
    		//3 is half the distance between the wheels
    		Wheel leftWheel = WheeledChassis.modelWheel(Motor.A, d).offset(-y);
    		Wheel rightWheel = WheeledChassis.modelWheel(Motor.B, d).offset(y);
    		Chassis chassis = new WheeledChassis(new Wheel[] {leftWheel, rightWheel}, WheeledChassis.TYPE_DIFFERENTIAL); 
    		pilot = new MovePilot(chassis);
    		
    		//drive.travelAndRotate();
    		
    		pilot.setLinearSpeed(fastTravelSpeed);
    		pilot.setAngularSpeed(25);
    		
    		//while (Button.ESCAPE.isUp()) {
    		//pilot.arc(9.5, 360);
    		
    		
    		pilot.travel(s);
    		pilot.arc(y, 90);
    		pilot.travel(s);
    		pilot.arc(y, 90);
    		pilot.travel(s);
    		pilot.arc(y, 90);
    		pilot.travel(s);
    		pilot.arc(y, 90);
    		
    		pilot.stop();
    }
}
