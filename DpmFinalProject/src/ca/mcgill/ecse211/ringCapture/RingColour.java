package ca.mcgill.ecse211.ringCapture;

import java.util.HashMap;
import java.util.Map;

import ca.mcgill.ecse211.navigation.MainClass;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class RingColour {
	
	//private static final Port csPort = LocalEV3.get().getPort("S3"); //change to proper port
	
	private SampleProvider ring_color_sample_provider;
	private float[] color_samples;

	private int redS, greenS, blueS;
	
	//Normalized colors
	private Map<String, int[]> colorMap = new HashMap<String, int[]>();
	private int[] greenRing = {52988, 115452, 15269, 13376, 26599, 1532}; // Rm, Gm,Bm, Rsd, Gsd, Bsd values (times 10^6 for each)
	private int[] orangeRing = {111483, 36549, 7096, 17996, 6088, 999};
	private int[] blueRing = {26097, 119187, 79725, 7591, 25473, 8012};
	private int[] yellowRing = {148692, 107749, 18901, 35384, 23879, 1561};
	
	/**
	 * Create new color sensor and assign keys to the colorMap
	 */
	public RingColour() {
		
		SensorModes ColorSensor = MainClass.ringSensor;
		ring_color_sample_provider = ColorSensor.getMode("RGB");
		this.color_samples = new float[ring_color_sample_provider.sampleSize()];
		colorMap.put("Green", greenRing);
		colorMap.put("Orange", orangeRing);
		colorMap.put("Blue", blueRing);
		colorMap.put("Yellow", yellowRing);
	}
	
	/**
	 * Assign detected R,G, and B values to an array
	 */
	public void fetchLightData() {
		ring_color_sample_provider.fetchSample(color_samples, 0);
		this.redS = (int) (color_samples[0] * 1000000);
		this.greenS = (int) (color_samples[1] * 1000000);
		this.blueS = (int) (color_samples[2] * 1000000);
	}
	
	/**
	 * Check if colour is witing 2 std deviations from mean
	 * @param colour
	 * @return true if detected color is within 2 std deviations
	 */
	public boolean colourDetected(String colour) {
		
		fetchLightData();
		
		int[] searchColorVal = colorMap.get(colour);
		
		if (Math.abs(searchColorVal[0] - this.redS) <= (int) (2 * searchColorVal[3]) &&
				Math.abs(searchColorVal[1] - this.greenS) <= (int) (2 * searchColorVal[4]) &&
				Math.abs(searchColorVal[2] - this.blueS) <= (int) (2 * searchColorVal[5]))
			{
				return true;
			}else {
				return false;
			}
	}
}
