import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.Font;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.utility.Delay;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;


public class SensorTest {

	public static void main(String[] args) {

        //System.out.println("Running...");
		Brick brick = BrickFinder.getDefault();
	    Port s2 = brick.getPort("S2");
	    EV3ColorSensor sensor = new EV3ColorSensor(s2);
	    SampleProvider rgb = sensor.getRGBMode();
	    float[] sample=new float[3];
	    rgb.fetchSample(sample, 0);
	    System.out.println("RGB="+sample[0]+" "+sample[1]+" "+sample[2]);
	    System.out.println("Press enter");
	    brick.getKey("Enter").waitForPressAndRelease();
		Button.LEDPattern(4);
        Sound.setVolume(1);
        Sound.beepSequenceUp();
        rgb.fetchSample(sample, 0);
	    System.out.println("RGB="+sample[0]+" "+sample[1]+" "+sample[2]);
	    System.out.println("Press enter");
	    brick.getKey("Enter").waitForPressAndRelease();
		Button.LEDPattern(4);
        Sound.setVolume(1);
        Sound.beepSequenceUp();
        rgb.fetchSample(sample, 0);
	    System.out.println("RGB="+sample[0]+" "+sample[1]+" "+sample[2]);
	    System.out.println("Press enter");
	    brick.getKey("Enter").waitForPressAndRelease();
		Button.LEDPattern(4);
        Sound.setVolume(1);
        Sound.beepSequenceUp();
        
        
        
        
        /*
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
        }*/
        
        Sound.beepSequence();
        Delay.msDelay(500);
        Button.LEDPattern(0);

	}

}
