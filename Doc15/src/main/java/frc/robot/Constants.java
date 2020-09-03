package frc.robot;

public class Constants {

    public static final double LOOPER_DT = 0.005; 

    //GoGoGadget
    //Front Actuator current under load: 22.5 A
    public static final int FRONT_NEO = 32;
    public static final int REAR_NEO = 31;

    public static final int RIGHT_GADGET_DRIVE_VICTOR = 8;
    public static final int LEFT_GADGET_DRIVE_VICTOR = 6;

    public static final double PITCH = 1.0 / 16.0; //1 in per 16 rev

    public static final int ULTRA_SONIC_PORT = 1;

    public static final double MAX_LENGTH = 21.0;
    public static final double MIN_LENGTH = 0.0;
    public static final double LEVEL_2_LENGTH = 8.0;
    public static final double MAX_GEARED_SPEED = 5676.0 * PITCH / 60.0; //5.9125

    public static final double R_ACTUATOR_MAX_LIN_VEL = 5.535; //in / s
    public static final double ACTUATOR_LIN_ACC = 20.0; //in / s*s

    public static final double ACTUATOR_MAX_ALLOWED_LIN_VEL = 5.0529; // in / s
    public static final double ACTUATOR_NO_CONTACT_LIN_VEL = MAX_GEARED_SPEED; //in / s
    public static final double ACTUATOR_LINEAR_ACCEL = 20.0; //in / s*s
    public static final double FAST_ACTUATOR_LINEAR_VEL = 0.9 * ACTUATOR_MAX_ALLOWED_LIN_VEL;
    public static final double SLOW_ACTUATOR_LINEAR_VEL = 50.0 * PITCH;

    public static final double FINAL_DESCENT_HEIGHT = 0.0;
    public static final double STARTING_DESCENT_HEIGHT = 0.0;

    //F means front, R means rear
    public static final double FRONT_GADGET_KV = 1.0 / ACTUATOR_MAX_ALLOWED_LIN_VEL;
    public static final double FRONT_GADGET_KP = 0.0;
    public static final double FRONT_GADGET_KI = 0.0;
    public static final double FRONT_GADGET_KD = 0.0;

    public static final double REAR_GADGET_KV = 0.0;//1.0 / ACTUATOR_MAX_ALLOWED_LIN_VEL;
    public static final double REAR_GADGET_KP = 1.0;
    public static final double REAR_GADGET_KI = 0.0;
    public static final double REAR_GADGET_KD = 0.0;

    public static final double GADGET_TOLERANCE = 0.5;

    public static final int PROXIMITY_BANNER = 0;

    //42 ticks / rev for NEO

    //VisionProcessor
    public static final double FOCAL_LENGTH = 0.0;
    public static final int WIDTH = 320; //pixels
    public static final int LENGTH = 240; //pixels

    public static final double CAM_X_OFFSET = 0.0;
    public static final double CAM_Y_OFFSET = 0.0;

    public static final double h1 = 5.25;//5.25; //in
    public static final double h2 = 38.5;//33.0 + 21.0 / 32.0;
    public static final double h3 = 31.25;
    public static final double a1 = 23.98; // deg
    

    //Drive
    public static final double WHEEL_BASE = 2.125; //ft
    //These are all incorrect now
    public static final double KS = 3.26; //Volts 
    public static final double KV = 0.0555;// Volts / (ips) 
    public static final double KA = 0.0511; //Volts / (ip(s^2))

    public static final double WHEEL_DIAMETER = 6.0; //in

    public static final int CAN_TIMEOUT_MS = 10; //for on the fly updates
    public static final int LONG_CAN_TIMEOUT_MS = 100; //for constructors

    public static final int DRIVE_LEFT_ENCODER_1 = 14;
    public static final int DRIVE_LEFT_ENCODER_2 = 15;
    public static final int DRIVE_RIGHT_ENCODER_1 = 12;
    public static final int DRIVE_RIGHT_ENCODER_2 = 13;
    //units are ft
    public static final double DRIVE_MAX_SPEED = 13.473;//17.246;//13.12;// ft / s//157.477;//23.95 * 12.0;//67.86 * 12.0;//0.8 * 300.0;//55.0 * 12.0;//67.86 * 12.0; //in / s or about 20.68 m / s
    public static final double DRIVE_MAX_ACC = 10.;//13.209;//14.372;//12.68 for new gearboxes //15.38; //possibly 10 in / s ^ 2//171.037;//16.15 * 12.0; //in /s ^ 2
    public static final double DRIVE_MAX_JERK = 60.0;//ft / s^30.0; //in / s^3

    public static final double DRIVE_ALLOWED_SPEED = 0.7 * DRIVE_MAX_SPEED;
    public static final double TEST_ALLOWED_SPEED = 0.3 * DRIVE_MAX_SPEED;

    public static final int DRIVE_IMU = 0;

    public static final int SHIFTER_SOLENOID = 0;

    public static final int LEFT_FRONT_DRIVE_VICTOR = 5;
    public static final int LEFT_REAR_DRIVE_VICTOR = 1;
    public static final int RIGHT_FRONT_DRIVE_VICTOR = 9;
    public static final int RIGHT_REAR_DRIVE_VICTOR = 0;

    public static final double TRAJECTORY_FOLLOWER_KP = 0.23;//0.8;
    public static final double TRAJECTORY_FOLLOWER_KI = 0.0;
    public static final double TRAJECTORY_FOLLOWER_KD = 0.005;//0.05;//0.05
    public static final double TRAJECTORY_FOLLOWER_KV = 0.043;//1.0 / DRIVE_MAX_SPEED;
    public static final double TRAJECTORY_FOLLOWER_KA = 0.0001;
    public static final double TRAJECTORY_FOLLOWER_KW = 0.03;
    //Needs retuning
    public static final double HEADING_CONTROLLER_KP = 0.045;//0.0095 //0.045;//0.03; 0.025
    public static final double HEADING_CONTROLLER_KI = 0.08;//0.0004;//0.004;//0.006//0.112;//0.06; //0.03
    public static final double HEADING_CONTROLLER_KD = 0.002;//0.0015;//0.00001;//0.00001;//0.00004//0.0015;//0.002;


    //Elevator
    public static final double ELEVATOR_MIN = 0.0;
    public static final double ELEVATOR_MAX = 46.0;//34.0;//60.0; // 62.0 inches abs max, pref 60.0 in

    public static final double LOW = 0.0;
    public static final double MID = 21.22; //21.0 maybe add 2.0 inches
    public static final double HIGH = 43.36;//43.1 probably needs to be at
    public static final double PLAYER = 13.0;

    public static final int LEFT_ELEVATOR_VICTOR = 7;
    public static final int RIGHT_ELEVATOR_VICTOR = 4;

    public static final int LEFT_ELEV_ENC_1 = 10;
    public static final int LEFT_ELEV_ENC_2 = 11;
    public static final int RIGHT_ELEV_ENC_1 = 6;
    public static final int RIGHT_ELEV_ENC_2 = 7;

    //Autotuned with simulation as of 2/13/2019 punishes position error of 0.01 in, velocity error of 0.1 in /s
    //and punishes control effort of 13 volts
    public static final double ELEV_KP = 7.2 * 0.0254; //0.1
    public static final double ELEV_KI = 0.0;
    public static final double ELEV_KD = 0.00163 * LOOPER_DT * 0.0254; //It is multiplied by looper_dt to cancel division by dt in ProfileController
    public static final double ELEV_KV = 1.0 / 60.629;

    public static final double ELEV_MAX_LIN_VEL = 60.629; //in / s
    public static final double ELEV_MAX_LIN_ACC = 80.0; //in / s ^2
    public static final double ELEV_MAX_ALLOWED_LIN_VEL = .9 * ELEV_MAX_LIN_VEL;

    public static final double ELEV_TOLERANCE = 0.5;

    //Pulley diameter times 3 since each rotation moves the inner stage up 3 times.
    public static final double PULLEY_DIAMETER = 2.0 * (0.8755 - 0.16) * 3.0; //inches

    //Intake
    public static final int INTAKE_VICTOR = 3;
    public static final int RETRACTOR_DOUBLE_SOLENOID_1 = 1;
    public static final int RETRACTOR_DOUBLE_SOLENOID_2 = 2;
    public static final int PANEL_SOLENOID = 3;
    public static final int INTAKE_BAR_EXTENDER_SOL = 4;
    public static final int INTAKE_BANNER = 2;
    public static final int INTAKE_LIMIT_SWITCH = 0;

    //Misc.
    public static final int MOLLY_SOL = 7;
    
}