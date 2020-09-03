package frc.util;

import java.util.Arrays;
import java.util.List;

import trajectory_lib.Waypoint;

/**
 * This class is only to contain crucial field measurements for the 2019 game. Essentially, 
 * every variable should be public static final (constant).
 */
public class FieldMeasurements {

    // field is rectangular! :D

    public static final double width = 27.0; //ft
    public static final double length = 54.0; //ft

    public static final double tapeWidth = 2.0; //in
    public static final double alignmentTapeLength = 1.5; //ft

    public static final double wallToHabLine = 94.0;//7.0 + (11.5 / 12.0);//7 ft 11.5 in
    public static final double wallToRocketStation2 = 9.0; //ft
    public static final double wallToRocketStation1 = 6.3; //ft
    public static final double wallToRocketStation3 = 11.7; //ft
    public static final double wallToCargoShip = 94.0 + 124.95; //in

    public static final double wallToCargoShipSide1 = 11 + 8.0/12.0; //ft
    public static final double wallToCargoShipSide2 = wallToCargoShipSide1 + 22.0 / 12.0; //ft
    public static final double wallToCargoShipSide3 = wallToCargoShipSide2 + 22.0 / 12.0; //ft
    public static final double allianceStationLength = 10.0; //ft
    public static final double wallToAllianceLine = 17.0 - 0.75; //ft
    //public static final double wallToCargoShip = 7.0 + (5.5 / 12.0); //ft
    public static final double level1Width = 47.25;
    public static final double cargoLeftX = 126.0 / 12.0;
    public static final double cargoLeftY = 9.5 / 12.0;

    //Cargo Left Panel
    public static final List<Waypoint> pathToCargoLeft = Arrays.asList(new Waypoint(0.0, 0.0, 0.0), new Waypoint( cargoLeftX, cargoLeftY, 0.0));
    public static final List<Waypoint> pathToCargoRight = Arrays.asList(new Waypoint(0.0, 0.0, 0.0), new Waypoint( cargoLeftX, -cargoLeftY, 0.0));
    
}