// Lab2.java
package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.localization.LightLocalizer;
import ca.mcgill.ecse211.localization.UltrasonicLocalizer;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.ringCapture.ArmController;
import ca.mcgill.ecse211.wifi.Wifi;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/***
 * This class runs all the methods required to complete the course.
 * 
 * @authorAbedAtassi
 * @authorHyunSuAn
 */
public class MainClass {

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final NXTRegulatedMotor armMotor = new NXTRegulatedMotor(LocalEV3.get().getPort("B"));
	public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
	public static final EV3ColorSensor lineSensor = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
	public static final EV3ColorSensor ringSensor = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
	public static final EV3GyroSensor gyroSensor = new EV3GyroSensor(LocalEV3.get().getPort("S4"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.07;
	public static final double TRACK = 10.53;
	public Odometer odometer;
	public static ArmController armController;
	public static Navigation navigation;
	public static Pilot pilot;

	/***
	 * Main
	 * 
	 * 
	 * @param args
	 * 
	 * @throws OdometerExceptions
	 */
	public static void main(String[] args) throws OdometerExceptions {

		// Sets up colour sensor and array holding data
		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, gyroSensor, TRACK, WHEEL_RAD);
		// ObstacleAvoidance obstacleAvoidance = new ObstacleAvoidance();
		Display odometryDisplay = new Display(lcd);
		UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(leftMotor, rightMotor, TRACK, WHEEL_RAD);

		Wifi wifi = new Wifi();
		navigation = new Navigation(leftMotor, rightMotor, TRACK, WHEEL_RAD, wifi);
		LightLocalizer lightLocalizer = new LightLocalizer(leftMotor, rightMotor, TRACK, WHEEL_RAD,
				gyroSensor, navigation);
		pilot = new Pilot(leftMotor, rightMotor, TRACK, WHEEL_RAD, wifi, navigation, lightLocalizer);
		armController = new ArmController(armMotor, leftMotor, rightMotor, WHEEL_RAD, TRACK);

		// Start odometer and display threads
		Thread odoThread = new Thread(odometer);
		odoThread.start();
		Thread odoDisplayThread = new Thread(odometryDisplay);
		odoDisplayThread.start();

		// Methods to complete the course
		usLocalizer.run();
		lightLocalizer.run();
		Sound.beep();
		Sound.beep();
		Sound.beep();
		gyroSensor.reset();
		pilot.run();
		armController.captureRing();
		pilot.travelBackToTunnel();
		pilot.travelBackThroughTunnel();
		pilot.travelBackToStartingCorner();
		Sound.beep();
		Sound.beep();
		Sound.beep();
		Sound.beep();
		Sound.beep();

		System.exit(0);
	}
}