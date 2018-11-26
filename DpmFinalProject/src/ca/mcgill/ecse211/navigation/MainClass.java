// Lab2.java
package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.localization.FinalLightLocalizer;

import ca.mcgill.ecse211.localization.UltrasonicLocalizer;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.ringCapture.ArmController;
import ca.mcgill.ecse211.wifi.Wifi;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

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
	public static final double TRACK = 10.6; // 66
	public Odometer odometer;
	public static ArmController armController;
	public static Navigation navigation;
	public static Pilot pilot;

	public static void main(String[] args) throws OdometerExceptions {
		int buttonChoice;
		// Sets up colour sensor and array holding data

		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, gyroSensor, TRACK, WHEEL_RAD);
		// ObstacleAvoidance obstacleAvoidance = new ObstacleAvoidance();
		Display odometryDisplay = new Display(lcd);
		ColorDisplay colorDisplay = new ColorDisplay(lcd);
		UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(leftMotor, rightMotor, TRACK, WHEEL_RAD);

		FinalLightLocalizer finalLightLocalizer = new FinalLightLocalizer(leftMotor, rightMotor, TRACK, WHEEL_RAD,
				gyroSensor);
		Wifi wifi = new Wifi();
		navigation = new Navigation(leftMotor, rightMotor, TRACK, WHEEL_RAD, wifi);
		pilot = new Pilot(leftMotor, rightMotor, TRACK, WHEEL_RAD, wifi, navigation, finalLightLocalizer);
		armController = new ArmController(armMotor, leftMotor, rightMotor, WHEEL_RAD, TRACK);

		// Start odometer and display threads
		Thread odoThread = new Thread(odometer);
		odoThread.start();
		Thread odoDisplayThread = new Thread(odometryDisplay);
		odoDisplayThread.start();

		//usLocalizer.run();

		//FinalLightLocalizer.run();
		//gyroSensor.reset();
		//Sound.beep();
		pilot.run();
		Sound.beep();
		//FinalLightLocalizer.run();
		// lightLocalizer.run();
		// Sound.beep();

		// Sound.beep();
		// armController.run();

		// }

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}
}