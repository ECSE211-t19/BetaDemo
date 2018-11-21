package ca.mcgill.ecse211.wifi;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;


public class Wifi {

  // ** Set these as appropriate for your team and current situation **
  private static final String SERVER_IP = "192.168.2.26";
  private static final int TEAM_NUMBER = 19;
  private Map data; 
  private int team;
  // Enable/disable printing of debug info from the WiFi class
  private static final boolean ENABLE_DEBUG_WIFI_PRINT = false;

  public Wifi () {
	  this.getMapData();
  }
  
  @SuppressWarnings("rawtypes")
  public void getMapData() {

    // Initialize WifiConnection class
    WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

    // Connect to server and get the data
    try {
     
      data = conn.getData();
      

    } catch (Exception e) {
      System.err.println("Error: " + e.getMessage());
    }

 
  }
  /**
	 * Gets the color of the team. 
	 * 
	 * @return  color for the team 
	 */
  public int getTeam() {
		team = ((Long) data.get("RedTeam")).intValue();
		if ( team == 19) {
			return 1;
		} else { 
			return 0;
		} 
	}

	/**
	 * Gets the starting corner of the given team.
	 * 
	 * @return  the starting corner of the team.
	 */
	public int getStartingCorner(int team) {
		team = getTeam();
		if (team == 1) {

			return ((Long) data.get("RedCorner")).intValue();
		} else if (team == 0) {

			return ((Long) data.get("GreenCorner")).intValue();
		}
		return -1;
	}

	/**
	 * Gets the starting corner coordinates 
	 *
	 * @return start
	 */
	public int[] getStartingCornerCoords() {
		int[] coords = { 0, 0 };
		team = getTeam();
		switch (getStartingCorner(team)) {
		case 0:
			coords[0] = 1;
			coords[1] = 1;
			break;
		case 1:

			coords[0] = 7;
			coords[1] = 1;
			break;
		case 2:
			coords[0] = 14;
			coords[1] = 8;
			break;
		case 3:
			coords[0] = 1;
			coords[1] = 8;
			break;
		}
		return coords;
	}
	
	/**
	 * Gets the coordinates of the green zone.
	 * 
	 * @return greenZone
	 */
	public int[][] getGreenZone() {
	
		int llx = ((Long) data.get("Green_LL_x")).intValue();
		int lly = ((Long) data.get("Green_LL_y")).intValue();
		int urx = ((Long) data.get("Green_UR_x")).intValue();
		int ury = ((Long) data.get("Green_UR_y")).intValue();

	
		int[][] greenZone = { { llx, lly }, { urx, lly }, { urx, ury },{ llx, ury } };

		return greenZone;
	}

	
	/**
	 * Gets the coordinates of the red zone.
	 * 
	 * @return coordinates of the red zone
	 */
	public int[][] getRedZone() {
		
		
		int lLX = ((Long) data.get("Red_LL_x")).intValue();
		int lLY = ((Long) data.get("Red_LL_y")).intValue();
		int uRX = ((Long) data.get("Red_UR_x")).intValue();
		int uRY = ((Long) data.get("Red_UR_y")).intValue();

		
		int[][] redZone = { { lLX, lLY }, { uRX, lLY }, { uRX, uRY },
				{ lLX, uRY } };

		return redZone;
	}
	
	
	/**
	 * Gets the coordinates of the tunnel
	 * 
	 * @return greenTunnelZone
	 */
	public int[][] getTunnel() {
		int llx, lly, urx, ury;
		team = getTeam();
		switch (team) {
		case 1:
			
			llx = ((Long) data.get("TNR_LL_x")).intValue();
			lly = ((Long) data.get("TNR_LL_y")).intValue();
			urx = ((Long) data.get("TNR_UR_x")).intValue();
			ury = ((Long) data.get("TNR_UR_y")).intValue();

			
			int[][] redTunnel = { { llx, lly }, { urx, lly },{ urx, ury }, { llx, ury } };

			return redTunnel;

		case 0:
			LCD.drawString("green" , 0, 5);

			llx = ((Long) data.get("TNG_LL_x")).intValue();
			lly = ((Long) data.get("TNG_LL_y")).intValue();
			urx = ((Long) data.get("TNG_UR_x")).intValue();
			ury = ((Long) data.get("TNG_UR_y")).intValue();

			
			int[][] greenTunnel = { { llx, lly }, { urx, lly },{ urx, ury }, { llx, ury } };

			return greenTunnel;
			
		default: return null;
		}
	}

	/**
	 * Gets the search zone of the specified team
	 *  
	 * @return greenSearchZone
	 */
	public int[][] getSearchZone() {
		int llx, lly, urx, ury;
		
		llx = ((Long) data.get("Island_LL_x")).intValue();
		lly = ((Long) data.get("Island_LL_y")).intValue();
		urx = ((Long) data.get("Island_UR_x")).intValue();
		ury = ((Long) data.get("Island_UR_y")).intValue();
		
		int[][] greenSearchZone = { { llx, lly }, { urx, lly },{ urx, ury }, { llx, ury } };

		return greenSearchZone;
	}
	
	/**
	 * Gets the ring set coordinates
	 * 
	 * @return 1-D ringSet
	 */
	public int[] getRingSet() {
		int tr_x,tr_y,tg_x,tg_y;

		switch (team) {
		case 1:
			
			tr_x = ((Long) data.get("TR_x")).intValue();
			tr_y = ((Long) data.get("TR_y")).intValue();

			int[] redRingSet = {tr_x, tr_y};

			return redRingSet;

		case 0:
		
			tg_x = ((Long) data.get("TG_x")).intValue();
			tg_y = ((Long) data.get("TG_y")).intValue();

			int[]greenRingSet = {tg_x,tg_y};

			return greenRingSet;

		default: return null;
		}
	}

	
	/**
	 * Determines the orientation of the tunnel
	 *
	 * @return boolean 
	 */
	public boolean isTunnelVertical() {
		
		team = getTeam();
		int llx = ((Long) data.get("Island_LL_x")).intValue();
		
		switch (team) {
		case 1:
			int urxR = ((Long) data.get("TNR_UR_x")).intValue();
			if (urxR <= llx) {
				return true;
			} else {
				return false;
			}
		case 0:
			int urxG = ((Long) data.get("TNG_UR_x")).intValue();
			if (urxG <= llx) {
				return true;
			} else {
				return false;
			}
		default: return false;
		}
	
		
		/*
		if (urx <= llx) {
			return true;
		} else {
			return false;
		}
		switch (team) {
		case 1:
			llx = ((Long) data.get("TNR_LL_x")).intValue();
			urx = ((Long) data.get("TNR_UR_x")).intValue();

			if(urx-llx == 1) {
				return true;
			}
			else {
				return false;
			}
		case 0:
			llx = ((Long) data.get("TNG_LL_x")).intValue();
			urx = ((Long) data.get("TNG_UR_x")).intValue();

			if(urx-llx == 1) {
				return true;
			}
			else {
				return false;
			}
			
		default: return false;

		}*/
	}
  
  
}
