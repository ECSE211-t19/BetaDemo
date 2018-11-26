package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.localization.FinalLightLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerData;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.wifi.Wifi;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Pilot implements Runnable {

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private double TRACK;
	private double WHEEL_RAD;
	public static final int FORWARD_SPEED = 200;
	private static final int ROTATE_SPEED = 150;
	private static final double TILE_WIDTH = 30.48;
	private Odometer odometer;
	private OdometerData odoData;
	Wifi wifi;
	int StartingCorner;
	int[] StartXY;
	int tunnel[][];
	int SearchZone[][];
	int tree[];
	int team;
	Navigation navigation;
	

	public Pilot(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, final double TRACK,
			final double WHEEL_RAD, Wifi wifi, Navigation navigation) throws OdometerExceptions {

		this.odometer = Odometer.getOdometer();
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;
		this.wifi = wifi;
		this.team = wifi.getTeam();
		this.StartingCorner = wifi.getStartingCorner(team);
		this.StartXY = wifi.getStartingCornerCoords();
		this.tunnel = wifi.getTunnel();
		this.tree = wifi.getRingSet();
		odoData = OdometerData.getOdometerData();
		//odoData.setXYT(0, 0, 0);

		this.navigation = navigation;

		
	}

	public void run() {
		try {
			this.odoData = Odometer.getOdometer();
		} catch (OdometerExceptions e1) {

			e1.printStackTrace();
		}
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.setAcceleration(200); // reduced the acceleration to make it smooth

		}

		switch (StartingCorner) {

		case 0:
			odoData.setXYT(StartXY[0] * TILE_WIDTH, StartXY[1] * TILE_WIDTH, 0);
			break;

		case 1:
			odoData.setXYT(StartXY[0] * TILE_WIDTH, StartXY[1] * TILE_WIDTH, 270);
			break;

		case 2:
			odoData.setXYT(StartXY[0] * TILE_WIDTH, StartXY[1] * TILE_WIDTH, 180);
			break;

		case 3:
			odoData.setXYT(StartXY[0] * TILE_WIDTH, StartXY[1] * TILE_WIDTH, 90);
			break;
		}
		travelToTunnel();
		travelThroughTunnel();
		//travelToRingSet();
	}

	/**
	 * Method to drive to the tunnel
	 */
	public void travelToTunnel() {

		if (wifi.isTunnelVertical()) {

			switch (StartingCorner) {

			case 0:

				navigation.travelTo((tunnel[0][0] + 0.5) * TILE_WIDTH, StartXY[1] * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[0][0] + 0.5) * TILE_WIDTH, (tunnel[1][1] - 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;

			case 1:

				LCD.drawString("tunnel " + tunnel[1][0] + " " + tunnel[1][1], 0, 4);
				navigation.travelTo((tunnel[1][0] - 0.5) * TILE_WIDTH, StartXY[1] * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[1][0] - 0.5) * TILE_WIDTH, (tunnel[1][1] - 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;
			case 2:

				navigation.travelTo((tunnel[1][0] - 0.5) * TILE_WIDTH, StartXY[1] * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[1][0] - 0.5) * TILE_WIDTH, (tunnel[3][1] + 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;
			case 3:

				LCD.drawString("ury" + tunnel[3][1], 0, 5);
				navigation.travelTo((tunnel[0][0] + 0.5) * TILE_WIDTH, StartXY[1] * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[0][0] + 0.5) * TILE_WIDTH, (tunnel[3][1] + 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;
			}
		} else {
			// LCD.drawString("lol", 0, 5);

			switch (StartingCorner) {

			case 0:
				navigation.travelTo((StartXY[0]) * TILE_WIDTH, (tunnel[3][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[0][0] - 1) * TILE_WIDTH, (tunnel[3][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;

			case 1:
				navigation.travelTo((StartXY[0]) * TILE_WIDTH, (tunnel[3][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[3][0] + 1) * TILE_WIDTH, (tunnel[3][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;

			case 2:
				navigation.travelTo((StartXY[0]) * TILE_WIDTH, (tunnel[3][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[3][0] + 1) * TILE_WIDTH, (tunnel[3][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;

			case 3:
				navigation.travelTo((StartXY[0]) * TILE_WIDTH, (tunnel[3][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[0][0] - 1) * TILE_WIDTH, (tunnel[3][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;

			}

		}
	}

	/**
	 * Method to drive through the tunnel
	 */
	public void travelThroughTunnel() {

		if (wifi.isTunnelVertical()) {
			//LCD.drawString("lol", 0, 3);

			switch (StartingCorner) {

			case 0:
				navigation.travelTo((StartXY[0]) * TILE_WIDTH, (tunnel[3][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[0][0] - 1) * TILE_WIDTH, (tunnel[3][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;

			case 1:
				navigation.travelTo((tunnel[1][0] - 0.5) * TILE_WIDTH, (tunnel[3][1] + 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[1][0] + 1) * TILE_WIDTH, (tunnel[3][1] + 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);

				return;

			case 2:
				navigation.travelTo((tunnel[1][0] - 0.5) * TILE_WIDTH, (tunnel[1][1] - 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[1][0] + 1) * TILE_WIDTH, (tunnel[1][1] - 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;

			case 3:
				navigation.travelTo((tunnel[0][0]) * TILE_WIDTH, (tunnel[3][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[0][0] - 1) * TILE_WIDTH, (tunnel[3][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;

			}
			
		}
		// assure that the robot is pointing 270
		else {
			LCD.drawString("no vert", 0, 4);
			navigation.travelTo((tunnel[3][0] + 1) * TILE_WIDTH, (tunnel[3][1] + 0.5) * TILE_WIDTH, FORWARD_SPEED,
					ROTATE_SPEED);
			navigation.travelTo((tunnel[3][0] + 1) * TILE_WIDTH, (tunnel[3][1] + 1) * TILE_WIDTH, FORWARD_SPEED,
					ROTATE_SPEED);
		}

	}

	/**
	 * Method to travel to the ringSet
	 * 
	 */
	public void travelToRingSet() {
		int uLX = tunnel[0][0];
		int uRX = tunnel[3][0];

		if ((uLX + uRX) / 2 > tree[0]) {
			LCD.drawString("left", 0, 5);
			navigation.travelTo(tree[0] + 1, tree[1], FORWARD_SPEED, ROTATE_SPEED);

		}

		else {
			LCD.drawString("right", 0, 5);
			navigation.travelTo(tree[0] - 1, tree[1], FORWARD_SPEED, ROTATE_SPEED);
		}

	}

	public void travelBackToTunnel() {
		
		navigation.travelTo(tree[0] + 1, tree[1], FORWARD_SPEED, ROTATE_SPEED);

	}

	public void travelBackThroughTunnel() {

		if (wifi.isTunnelVertical()) {
			LCD.drawString("lhoriz", 0, 3);

			navigation.travelTo((tunnel[1][0] + 0.5) * TILE_WIDTH, (tunnel[3][1] + 1) * TILE_WIDTH, FORWARD_SPEED,
					ROTATE_SPEED);
			navigation.travelTo((tunnel[1][0] + 1) * TILE_WIDTH, (tunnel[3][1] + 1) * TILE_WIDTH, FORWARD_SPEED,
					ROTATE_SPEED);

		}
		// assure that the robot is pointing 270
		else {
			LCD.drawString("no vert", 0, 4);
			leftMotor.rotate(-navigation.convertAngle(WHEEL_RAD, TRACK, 90), true);
			rightMotor.rotate(navigation.convertAngle(WHEEL_RAD, TRACK, 90), false);
			navigation.travelTo((tunnel[3][0] + 1) * TILE_WIDTH, (tunnel[3][1] + 0.5) * TILE_WIDTH, FORWARD_SPEED,
					ROTATE_SPEED);
			navigation.travelTo((tunnel[3][0] + 1) * TILE_WIDTH, (tunnel[3][1] + 1) * TILE_WIDTH, FORWARD_SPEED,
					ROTATE_SPEED);
		}

	}

	public void travelBackToStartingCorner() {

		navigation.travelTo(StartXY[0] , StartXY[1], FORWARD_SPEED, ROTATE_SPEED);
		
	}

}
