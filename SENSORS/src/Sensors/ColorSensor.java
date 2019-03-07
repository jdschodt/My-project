package Sensors;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class ColorSensor {
	private static EV3ColorSensor colorSensor;
	private static SampleProvider colorIDProvider;
	private static SampleProvider RGBProvider;
	private static float[] colorIDSample;
	private static float[] RGBSample;
	private static int	colorID;
	
	public ColorSensor(){
		Port s3 = LocalEV3.get().getPort("S3");
		colorSensor = new EV3ColorSensor(s3);
		colorIDProvider = colorSensor.getColorIDMode();
		RGBProvider=colorSensor.getRGBMode();
		colorIDSample = new float [colorIDProvider.sampleSize()];
		RGBSample=new float [RGBProvider.sampleSize()];
		
	}
	
	public int getColorID(){
		colorIDProvider.fetchSample(colorIDSample, 0);
		colorID=(int)colorIDSample[0];
		return colorID;
		}
	public float[] getRGB() {
		RGBProvider.fetchSample(RGBSample, 0);
		return RGBSample;
	}
	
	public void close() {
		colorSensor.close();
}



}
