package org.usfirst.frc.team1671.robot;

import org.ojalgo.matrix.PrimitiveMatrix;

import control.MatrixReader;
import jaci.pathfinder.followers.EncoderFollower;
import trajectory_lib.TrajectoryConfig;

public class RobotMap {
	public static final double kLooperDt = 0.005;
	public static final int kLongCANTimeoutMs = 100; //use for constructors
	
	//Measurements
	public static final double WHEELBASE = 28.0 / 12.0; //ft /s
	public static final double MAX_VELOCITY = 188.1;//188.1; //in / sec //Nope its actually 158.59984363915828
	public static final double MAX_ACCELERATION = 130.0;//516.3388520911849 * 0.225 / 0.05; //2422.937464; // in / sec^2 // Nope its actually 516.3388520911849 but worry later
	public static final double MAX_JERK = 500.0;//2319.478834377015 / (71.320564 - 71.301108); // in / sec^3 119216.63416823
	
	public static final double FOLLOWER_MAX_V = 0.7 * MAX_VELOCITY / 12.0; //ft/s
	public static final double FOLLOWER_MAX_A = 11.0; //ft/s^2
	public static final double FOLLOWER_MAX_J = 240.0; //ft/s^2
	
	public static final double ROBOT_WIDTH = 34.0 / 12.0; //ft
	public static final double ROBOT_LENGTH = 39.0 / 12.0; //ft
	
	public static final boolean DRIVE_LEFT_ENC_DIRECTION = true; //forward is true
	public static final boolean DRIVE_RIGHT_ENC_DIRECTION = false; //forward is false
	
	public static final double ARM_OUT = 11.0 / 12.0; //ft
	
	//Encoder measured 1114, while it should have been 1146
	
	public static final TrajectoryConfig config = new TrajectoryConfig(TrajectoryConfig.SAMPLES_FAST, RobotMap.kLooperDt, 
			RobotMap.FOLLOWER_MAX_V, RobotMap.FOLLOWER_MAX_A, RobotMap.FOLLOWER_MAX_J);
	
	//OI
	public static final boolean IS_STICK_WHEEL = false;
	public static final int BASE_JOYSTICK_PORT = 0;
	public static final int CO_JOYSTICK_PORT = 1;
	public static final int WHEEL_JOYSTICK_PORT = 2;
	public static final int STICK_JOYSTICK_PORT = 3;
	public static final double STICK_DEAD_ZONE = 0.1;
	
	//Drive
	public static final int LEFT_FRONT_VICTOR_PORT = 6;
	public static final int LEFT_MID_VICTOR_PORT = 5;
	public static final int LEFT_REAR_TALON_PORT = 4;
	public static final int RIGHT_FRONT_VICTOR_PORT = 2;
	public static final int RIGHT_MID_VICTOR_PORT = 3;
	public static final int RIGHT_REAR_TALON_PORT = 1;
	
	public static final int DRIVE_SHIFTER_FORWARD = 2;
	public static final int DRIVE_SHIFTER_REVERSE = 3;
	
	public static final int LEFT_ENCODER_1 = 12;
	public static final int LEFT_ENCODER_2 = 13;
	public static final int RIGHT_ENCODER_1 = 10;
	public static final int RIGHT_ENCODER_2 = 11;
	
	public static final double SIMPLE_KP = 0.01;
	public static final double SIMPLE_KI = 0.0;
	public static final double SIMPLE_KD = 0.0;
	public static final double SIMPLE_KV = 1.0 / (140.0 / 12.0);
	
	public static final double PATH_FOLLOWER_KP = 0.0;//0.05;
	public static final double PATH_FOLLOWER_KI = 0.0;
	public static final double PATH_FOLLOWER_KD = 0.00;//0.000001;
	public static final double PATH_FOLLOWER_KV = 1.0 / ( (MAX_VELOCITY) / 12.0 );
	public static final double PATH_FOLLOWER_KA = 0.007;//0.02; //start at < 0.01
	
	public static final double PATH_FOLLOWER_KW = 0.8 * (1.0 / 80.0);

	public static final double IN_PLACE_KP = 0.8 * (1.0 / 80.0);
	public static final double IN_PLACE_KI = 0.0;///007;
	public static final double IN_PLACE_KD = 0.00;
	// ehh, works, 85 is 90,
	//60 is 45
	
	
	public static final int IMU = 0;
	
	public static final double DIAMETER = 6.0; //inches
	
	//Elevator
	public static final int LEFT_TOP_ELEVATOR = 7;
	public static final int LEFT_BOTTOM_ELEVATOR = 0;
	public static final int RIGHT_TOP_ELEVATOR = 10;
	public static final int RIGHT_BOTTOM_ELEVATOR = 8;
	
	public static final int ELEVATOR_SHIFTER_FORWARD = 0;
	public static final int ELEVATOR_SHIFTER_REVERSE = 1;
	
	public static final int LEFT_ELEVATOR_ENC_1 = 14;
	public static final int LEFT_ELEVATOR_ENC_2 = 15;
	public static final int RIGHT_ELEVATOR_ENC_1 = 16;
	public static final int RIGHT_ELEVATOR_ENC_2 = 17;
	
	public static final double MAX_ELEV_VELOCITY = 61.492; //in / s
	public static final double MAX_ELEV_ACCEL = 139.852; //in / s^2
	public static final double ELEV_KP = 0.11;//0.04;
	public static final double ELEV_KI = 0.0;
	public static final double ELEV_KD = 0.0;//0.001;
	public static final double ELEV_KV = 1.0 / MAX_ELEV_VELOCITY;
	public static final double ELEV_TOLERANCE = 1.0;
	public static final double MAX_ELEV_HEIGHT = 67.0;
	public static final double MIN_ELEV_HEIGHT = 1.5;
	
	public static final double ELEV_ENC_RATE = (77.625-16.75) / (2007.750 - (-42.5));
	
	//Arm
	public static final int ARM_MOTOR = 9;
	
	public static final int ARM_ENCODER_1 = 18;
	public static final int ARM_ENCODER_2 = 19;
	public static final double ARM_ENCODER_RATE = 90.0 / 76.5;
	
	public static final double ARM_MAX_SPEED = 1.538 * 180.0 / Math.PI; //Theoretical
	public static final double ARM_MAX_ACCELERATION = 36.151 * 180.0 / Math.PI; //Theoretical
	public static final double ARM_MAX_ANGLE = 101.0;
	public static final double ARM_MIN_ANGLE = 0.0;

	public static final double ARM_KP = 0.214;//0.214 as of china;//0.025;
	public static final double ARM_KI = 0.0;//0.0;
	public static final double ARM_KD = 0.08;// this is the correct as of after Chinas;//0.3;
	public static final double ARM_KV = 1.0 / ARM_MAX_SPEED;
	public static final double ARM_TOLERANCE = 0.2942;
	
	//Don't worry about these for now, these are for future matrix operations.
	/*public static final PrimitiveMatrix A = MatrixReader.parseMatrix("src\\python\\arm\\A.csv");
	public static final PrimitiveMatrix B = MatrixReader.parseMatrix("src\\python\\arm\\B.csv");
	public static final PrimitiveMatrix C = MatrixReader.parseMatrix("src\\python\\arm\\C.csv");
	public static final PrimitiveMatrix D = MatrixReader.parseMatrix("src\\python\\arm\\D.csv");
	public static final PrimitiveMatrix K = MatrixReader.parseMatrix("src\\python\\arm\\K.csv");
	public static final PrimitiveMatrix Kff = MatrixReader.parseMatrix("src\\python\\arm\\Kff.csv");
	public static final PrimitiveMatrix L = MatrixReader.parseMatrix("src\\python\\arm\\L.csv");*/
	
	public static final PrimitiveMatrix ARM_U_MIN = PrimitiveMatrix.FACTORY.rows(new double[][] {
		{-12.0}
	});
	public static final PrimitiveMatrix ARM_U_MAX = PrimitiveMatrix.FACTORY.rows(new double[][] {
		{12.0}
	});
	
	//Intake
	public static final int INTAKE_MOTOR_LEFT = 11;
	public static final int INTAKE_MOTOR_RIGHT = 12;
	
	public static final int INTAKE_PISTON_BOTTOM = 4;
	public static final int INTAKE_PISTON_TOP = 5;
	
	public static final int INTAKE_BANNER = 0;
	
	//Superstructure
	public static final double SCALE_HEIGHT = 65.0;
	public static final double SWITCH_HEIGHT = 30.0;
	public static final double STOW_HEIGHT = 0.0;
	public static final double SAFE_CUBE_HEIGHT = 53.0;
	public static final double MANUAL_MIN_HEIGHT = 1.0;
	public static final double MANUAL_MAX_HEIGHT = 63.5;

	public static final double SCALE_ANGLE = 45.0;
	public static final double INTAKE_ANGLE = 101.0; //Also is switch angle
	public static final double STOW_ANGLE = 0.0;
	public static final double STOW_ANGLE_CUBE = 25.0;
	
	//LEDs
	public static final int BAUD_RATE = 115200;
	public static final int LEDsDIO1 = 0;
	public static final int LEDsDIO2 = 1;
	public static final int LEDsDIO3 = 2;
	
	
}
