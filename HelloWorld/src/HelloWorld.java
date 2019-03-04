import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.Font;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.utility.Delay;

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
    }
}
