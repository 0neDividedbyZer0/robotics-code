/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.lang.Throwable;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import frc.auto.AutoModeExecuter;

import frc.loops.Looper;
import frc.statemachines.GadgetState;
import frc.statemachines.GadgetStateMachine.GadgetWantedState;
import frc.subsystems.Drive;
import frc.subsystems.Elevator;
import frc.subsystems.GoGoGadget;
import frc.subsystems.Intake;
import frc.subsystems.LimelightProcessor;
import frc.subsystems.Superstructure;
import frc.subsystems.Drive.DriveState;
import frc.subsystems.Intake.IntakeState;
import frc.util.ArcadeDriveHelper;
import frc.util.CheesyDriveHelper;
import frc.util.DriveSignal;
import frc.util.SlewRateLimiter;
import frc.util.Util;

import trajectory_lib.AutoTrajectory;
import trajectory_lib.Trajectory;
import trajectory_lib.TrajectoryGenerator;
import trajectory_lib.Waypoint;
import frc.loops.CrashTracker;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot {
  //First you need every subsystem to be created
	//Every subsystem is a singleton, meaning only one version of it exists
  //Calling each subsystem from here is the same across all of the robot classes
  //ex: private Drive drive = Drive.getInstance();
  //private Drive drive = Drive.getInstance();
  private AutoModeExecuter autoModeExecuter = new AutoModeExecuter();
  private Drive drive = Drive.getInstance();
  private Intake intake = Intake.getInstance();
  private Elevator elevator = Elevator.getInstance();
  private GoGoGadget gadget = GoGoGadget.getInstance();
  private Superstructure superstructure = Superstructure.getInstance();
  private LimelightProcessor processor = LimelightProcessor.getInstance();

  //A Subsystem Manager for Convenience
	//Each subsystem has an internal looper. Adding or removing a subsystem
	//From this list will effectively add or remove functionality from the subsystem.
  private final SubsystemManager subsystemManager = new SubsystemManager(Arrays.asList(Drive.getInstance(),GoGoGadget.getInstance(), Elevator.getInstance(),
    Superstructure.getInstance(), Intake.getInstance(), LimelightProcessor.getInstance()));
    //Elevator.getInstance(), GoGoGadget.getInstance(), Superstructure.getInstance()));
  
  //Some 254 stuff to run faster loops at 200 hz, or 0.005 seconds
  Looper internalLooper = new Looper();
  
  private static XboxController base = new XboxController(0);
  private static XboxController co = new XboxController(1);
  private static Stick stick = new Stick(2);
  private static Wheel wheel = new Wheel(3);

  private static AxisGreater baseRightTrigger = new AxisGreater(base, 3, 0.3);
  private static AxisGreater baseLeftTrigger = new AxisGreater(base, 2, 0.3);

  private static AxisGreater coRightTrigger = new AxisGreater(co, 3, 0.3);
  private static AxisGreater coLeftTrigger = new AxisGreater(co, 2, 0.3);

  private double midLevel = Constants.PLAYER;
  
  private ArcadeDriveHelper arcadeDriveHelper = new ArcadeDriveHelper();
  private CheesyDriveHelper cheesyDriveHelper = new CheesyDriveHelper();
  private SlewRateLimiter slewRateLimiter = new SlewRateLimiter(0.1);
  private SlewRateLimiter turnLimiter = new SlewRateLimiter(0.2);

  private boolean isStickAndWheel = false;

  private boolean toggleHatch = false;
  private boolean toggleGoGoGadget = false;
  private boolean toggleIntake = false;
  private boolean autoGadget = false;
  private boolean autoDrive = false;
  private boolean ballAutoDrive = false;
  private boolean level2 = false;

  private boolean ballDetected = false;

  private double autoDriveTimestamp = 0.0;

  private AutoDriveData data = new AutoDriveData();

  //private double t_x, t_y, t_v, t_s, dist, ballDist;

  //private double savedSkew = 0.0;
  //private double savedDist = 0.0;

  private int LEDCounter = 0;
  private int targetCounter = 0;

  private boolean beginFlash = false;
  private boolean beginAcquireTarget = false;

  private GadgetSequence gadgetAutoStep = GadgetSequence.OFF;

  private double autoGadgetTimestamp = 0.0;

  private int ghettoReset = 0;
  private int ghettoReset1 = 0;

  /*NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("limelight");
  NetworkTableEntry ledMode = table.getEntry("ledMode");
  NetworkTableEntry webCam = table.getEntry("camMode");*/

  /*private double x = t_x
  private double y = t_y
  private double skew = t_s
  calculate distance later in code
  */
  private void setStickAndWheel(boolean on) {
    if(isStickAndWheel != on) {
      isStickAndWheel = on;
    }
  }

  //Note: Sensors are zeroed upon RobotInit, AutonomousInit.
  
  //Quickly repurposed for changing mid level on elevator
	public enum AutoMode {
    HUMAN_PLAYER, // start
    ROCKET_SHIP,
    CARGO_LEFT_PANEL_MODE,
    CARGO_RIGHT_PANEL_MODE,
    CARGO_LEFT_PANEL_LEVEL_2_MODE,
    CARGO_RIGHT_PANEL_LEVEL_2_MODE
  }

  public enum GadgetSequence {
    RAISE_BOTH,
    WAIT_BEFORE_DRIVE_FORWARD,
    DRIVE_FORWARD,
    WAIT_BEFORE_ADJUST,
    ADJUST, // Includes raising front
    DRIVE_ON,
    WAIT_BEFORE_RAISE_REAR,
    RAISE_REAR,
    WAIT_BEFORE_FINISH,
    FINISH,
    OFF
  }

  public enum AutoDriveSequence {
    PROCESS_ANGLE,
    TURN_HORIZONTAL,
    DRIVE_HORIZONTAL,
    TURN_VERTICAL,
    DRIVE_FORWARD,
    //ADJUST_ANGLE,
    //DRIVE_TO_TARGET,
    //ADJUST_TO_SKEW,
    FINISH,
    OFF
  }

  public enum GadgetLeg {
    FRONT,
    REAR,
    BOTH,
    RESET,
    NULL
  }
  
  private AutoMode autoMode = AutoMode.HUMAN_PLAYER;

  private GadgetLeg leg = GadgetLeg.BOTH;

  private AutoDriveSequence driveSequence= AutoDriveSequence.OFF;

  public Robot() {
    //sanic = new AnalogInput(0);
		CrashTracker.logRobotConstruction();
  }
  
  public void zeroSensors() {
		subsystemManager.zeroSensors();
	}

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    try {
      //NetworkTableEntry ledMode = table.getEntry("ledMode");
      //processor.setLedMode(3);
      //processor.setWebCamMode(1);
      subsystemManager.registerEnabledLoops(internalLooper);
      /*UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 320, 240, 15);
      MjpegServer cameraServer = new MjpegServer("serve_USB Camera 0", 0);
      cameraServer.setSource(camera);*/
			//camera.setResolution(640, 480);
      //camera.setFPS(30);

            
      
      
    } catch(Throwable t) {
      CrashTracker.logThrowableCrash(t);
			throw t;
    }
    zeroSensors();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    try {
			CrashTracker.logAutoInit();
      
      //Let's hope there isn't game data again
			//String gameData = DriverStation.getInstance().getGameSpecificMessage();
			
			System.out.println("Auto start timestamp: " + Timer.getFPGATimestamp());
			
			if (autoModeExecuter != null) {
        autoModeExecuter.stop();
      }
			
			zeroSensors();
			
			autoModeExecuter = null;
			
			internalLooper.start();
		  drive.setOpenLoop(DriveSignal.NEUTRAL);
			drive.setBrakeMode(true);
      drive.setHighGear(true);

      slewRateLimiter.reset();
      turnLimiter.reset();

      //processor.setWebCamMode(1);

      
      

			//Setting this to an auto will cause autonomous to operate
			
			//autoModeExecuter = new AutoModeExecuter();
			/*switch(autoMode) {
				case DO_NOTHING:
					//autoModeExecuter.setAutoMode(new DoNothingMode());
          break;
        case VELOCITY:
          //autoModeExecuter.setAutoMode(new CharacterizeMode());
          autoModeExecuter.setAutoMode(new TestDriveMode());
          break;
        case CARGO_LEFT_PANEL_MODE:
          autoModeExecuter.setAutoMode(new CargoLeftPanelMode());
          break;
        case CARGO_RIGHT_PANEL_MODE:
          autoModeExecuter.setAutoMode(new CargoRightPanelMode());
          break;
        case CARGO_RIGHT_PANEL_LEVEL_2_MODE:
          autoModeExecuter.setAutoMode(new CargoRightPanelLevel2Mode());
          break;
        case CARGO_LEFT_PANEL_LEVEL_2_MODE:
          autoModeExecuter.setAutoMode(new CargoLeftPanelLevel2Mode());
          break;
				default:
					System.out.println("Unexpected Auto mode, running Do Nothing");
					autoModeExecuter.setAutoMode(new DoNothingMode());
			}
      autoModeExecuter.start();*/
      

		} catch(Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    //if(autoMode == AutoMode.DO_NOTHING) {
      //operatorControls();
    //}
    

    allPeriodic();
  }



  @Override
	public void teleopInit() {
		try {
			CrashTracker.logTeleopInit();
			if(autoModeExecuter != null) {
				autoModeExecuter.stop();
			}
			
			internalLooper.start();
		  drive.setOpenLoop(DriveSignal.NEUTRAL);
			drive.setBrakeMode(false);
      drive.setHighGear(true);

      slewRateLimiter.reset();
      turnLimiter.reset();

      //processor.setWebCamMode(1); //This sets the webcam to vision processing
      //processor.setLedMode(2);
      
      //setStickAndWheel(SmartDashboard.getBoolean("Stick and Wheel", false));

      switch(autoMode) {
				case HUMAN_PLAYER:
					midLevel = Constants.PLAYER;
          break;
        case ROCKET_SHIP:
          midLevel = Constants.MID;
          break;
        default:
          midLevel = Constants.PLAYER;
      }
			
		} catch(Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
  } 
  //elev mode
  //left stick manual control elev
  //right trigger in
  //left trigger out
  //rb fold intake toggle
  //lb toggle mode
  //a-b-y l-m-h
  //x manual

  //GGG toggle lb
  //left stick drive/gadget
  //right stick turn/gadget drive
  //rb auto
  //start is reset
  //triggers control direction
  //a rear, y is front, b both

  public void operatorControls() {
    DriveSignal signal;
    boolean isHighGear;
    double throttle, turn, gadgetThrottle;
    boolean manualGadget;
    double height = elevator.getLeftEncoder();
    double throttleFactor = 1.0 - ((1.0 - 0.5) / (0.0 - 46.0) * height);
    double turnFactor = 1.0 - ((1.0 - 0.8) / (0.0 - 46.0) * height);
    
    
    
    /*if(isStickAndWheel) {
      throttle = stick.getThrottle();
      turn = wheel.getWheel();
      boolean isQuickTurn = stick.getButton(2);
      isHighGear = !stick.getButton(1);
      signal = cheesyDriveHelper.cheesyDrive(throttle, turn, isQuickTurn, isHighGear);
    } else {
      throttle = base.getY(Hand.kLeft);
      turn = base.getX(Hand.kRight);
      isHighGear = !baseShiftTrigger.get();
      signal = arcadeDriveHelper.arcadeDrive(throttle, turn);
    }*/
    base.getPOV();

    throttle = -throttleFactor * slewRateLimiter.limit(base.getY(Hand.kLeft)); //This is negative??
    turn = 0.9 * turnFactor * turnLimiter.limit(base.getX(Hand.kRight));
    
    isHighGear = !baseRightTrigger.get(); //putting ! before defaults to high gear
    gadgetThrottle = 0.0;
    if(!toggleGoGoGadget) {
      if(base.getBumperPressed(Hand.kRight)) {
        autoDrive = !autoDrive;
        if(processor.getTV() == 0.0) {
          autoDrive = false;
        } else {
          driveSequence = AutoDriveSequence.PROCESS_ANGLE;
          autoDriveTimestamp = Timer.getFPGATimestamp();
          /*double delta_x = 0.99 / 12.0 * processor.getDist() * Math.cos(Math.toRadians(drive.getHeading() - processor.getTX()));
          double delta_y = 0.99 / 12.0 * processor.getDist() * Math.sin(Math.toRadians(drive.getHeading() - processor.getTX()));
          //TODO: check if ts needs to be + or -
          List<Waypoint> path = Arrays.asList(new Waypoint(0.0, 0.0, drive.getHeading()), new Waypoint(delta_x, delta_y, Math.toRadians(drive.getHeading() - processor.getTX() - processor.getTS())));
          Trajectory trajectory = TrajectoryGenerator.generateQuinticHermiteSpline(drive.getConfig(), path);
          AutoTrajectory traj = TrajectoryGenerator.makeLeftRightTrajectories(trajectory, Constants.WHEEL_BASE);
          drive.setTrajectory(traj, false);*/
          
        }
        beginAcquireTarget = true;
        
        
        
        //savedSkew = t_s;
        //savedDist = dist;
        
      }
      
    }
    

    
    
    
    //4 is lb
    if(base.getBumperPressed(Hand.kLeft)) {
      toggleGoGoGadget = !toggleGoGoGadget;
    }
    ghettoResetBase();

    if(!toggleGoGoGadget) {
      autoGadget = false;
      gadgetAutoStep = GadgetSequence.OFF;
    }

    if(toggleGoGoGadget) {
      autoDrive = false;
      driveSequence = AutoDriveSequence.OFF;
    }

    if(toggleGoGoGadget) {
      throttle *= 0.75;
      gadgetThrottle = -base.getY(Hand.kRight);
      turn = 0.0;
      isHighGear = false;
      manualGadget = base.getBackButton();
      boolean startButton = base.getStartButtonPressed();
      
      if(base.getBumperPressed(Hand.kRight) || startButton) {
        if(startButton) {
          level2 = true;
        } else {
          level2 = false;
        }
        autoGadget = !autoGadget;
        if(autoGadget) {
          gadgetAutoStep = GadgetSequence.RAISE_BOTH;
          
        } else {
          gadgetAutoStep = GadgetSequence.OFF;
          
        }
        autoGadgetTimestamp = Timer.getFPGATimestamp();
        
      } 
      
        

      

      if(!Util.epsilonEquals(throttle, 0.0, 0.1) || !Util.epsilonEquals(gadgetThrottle, 0.0, 0.1) || manualGadget == true) {
        autoGadget = false;
        gadgetAutoStep = GadgetSequence.OFF;
      }
      //System.out.println(autoGadget);

      double wantedHeight;
      
      if(base.getAButton()) {
        leg = GadgetLeg.REAR;
      } else if(base.getYButton()) {
        leg = GadgetLeg.FRONT;
      } else if(base.getBButton()) {
        leg = GadgetLeg.BOTH;
      } else {
        leg = GadgetLeg.NULL;
      }

      if(leg != GadgetLeg.NULL) {
        autoGadget = false;
        gadgetAutoStep = GadgetSequence.OFF;
      }

      boolean retract, extend;

      extend = baseRightTrigger.get();
      retract = baseLeftTrigger.get();

      if(extend != false || retract != false) {
        autoGadget = false;
        gadgetAutoStep = GadgetSequence.OFF;
      }

      GadgetState observedState = gadget.getObservedState();
      
      wantedHeight = 0.0;
      double gadgetSignal;
      // code in gogogadget

      if(extend == retract) {
        //wantedGadgetState = GadgetWantedState.IDLE;
        //gadget.setRearMotor(0.0);
        gadgetSignal = 0.0;
        
      } else {
        if(extend) {
          wantedHeight = Constants.MAX_LENGTH;
          //gadget.setRearMotor(0.1);
          gadgetSignal = 0.1;
        } else {
          wantedHeight = Constants.MIN_LENGTH;
          //gadget.setRearMotor(-0.1);
          gadgetSignal = -0.1;
        }
        if(gadgetSignal != 0.0) {
          gadgetThrottle = 0.0;
        }

        
      }

      if(!autoGadget) {
        switch(leg) {
          case FRONT:
            if(manualGadget) {
              gadget.setWantedState(GadgetWantedState.MANUAL);
              gadget.setFrontMotor(2.0 * gadgetSignal);
            } else {
              if(gadget.getWantedState() == GadgetWantedState.MANUAL) {
                gadget.setFrontMotor(0.0);
              }
              gadget.setWantedState(GadgetWantedState.UNSYNCED);
            
              if(extend != retract) {
                gadget.setFrontGoal(wantedHeight);
              } else {
                gadget.setFrontGoal(gadget.getFrontEncoder());
              }
              gadget.setRearGoal(gadget.getRearEncoder());
            }
            
            
            
            break;
          case REAR:
            if(manualGadget) {
              gadget.setWantedState(GadgetWantedState.MANUAL);
              gadget.setRearMotor(1.675 * gadgetSignal);
            } else {
              if(gadget.getWantedState() == GadgetWantedState.MANUAL) {
                gadget.setRearMotor(0.0);
              }
              gadget.setWantedState(GadgetWantedState.UNSYNCED);
              if(extend != retract) {
                gadget.setRearGoal(wantedHeight);
              } else {
                gadget.setRearGoal(gadget.getRearEncoder());
              }
              gadget.setFrontGoal(gadget.getFrontEncoder());
            }
            
            
            
            break;
          case BOTH:
            if(manualGadget) {
              gadget.setFrontMotor(2.0 * gadgetSignal);
              gadget.setRearMotor(1.675 * gadgetSignal);
            } else {
              if(gadget.getWantedState() == GadgetWantedState.MANUAL) {
                gadget.setFrontMotor(0.0);
                gadget.setRearMotor(0.0);
              }
              gadget.setWantedState(GadgetWantedState.SYNCED);
            
              if(extend != retract) {
                gadget.setFrontGoal(wantedHeight);
                gadget.setRearGoal(wantedHeight);
              } else {
                gadget.setFrontGoal(gadget.getFrontEncoder());
                gadget.setRearGoal(gadget.getRearEncoder());
              }
            }
            
            
            
            break;
          case RESET:
            if(observedState.frontActuatorLength > observedState.rearActuatorLength) {
              wantedHeight = observedState.rearActuatorLength;
            } else if(observedState.frontActuatorLength <= observedState.rearActuatorLength) {
              wantedHeight = observedState.frontActuatorLength;
            }
            gadget.setWantedState(GadgetWantedState.SYNCED);
            gadget.setFrontGoal(wantedHeight);
            gadget.setRearGoal(wantedHeight);
            
            break;
          default:
            //gadget.setWantedState(GadgetWantedState.UNSYNCED);
            //gadget.setFrontGoal(gadget.getFrontEncoder());
            //gadget.setRearGoal(gadget.getRearEncoder());
            System.out.println("error");
            gadget.setWantedState(GadgetWantedState.MANUAL);
            gadget.setFrontMotor(0.0);
            gadget.setRearMotor(0.0);
            break;
        }
        gadget.setGadgetDrive(gadgetThrottle); //Won't run for now since unsynced wont let it.
        //System.out.println(leg);
      } 
      

      
      
      
    }
      

    /*if(co.getBumperPressed(Hand.kLeft)) {
      toggleHatch = !toggleHatch;
    }
    ghettoResetCo();*/

    

    

    
    

    Intake.IntakeState wantedIntakeState;
    wantedIntakeState = intake.new IntakeState();

    
    if(coRightTrigger.get()) {
      wantedIntakeState.velocity = 0.4;
    } else if(coLeftTrigger.get()) {
      wantedIntakeState.velocity = -0.6; //-0.5
    }
  
    if(co.getBumper(Hand.kRight)) {
      wantedIntakeState.eject = Util.epsilonEquals(intake.getUltraSonicSensor(), 4.0, 1.5);
      boolean currentEjector = intake.getEjectorBoolean();
      if(toggleHatch != true) {
        if(currentEjector) {
          beginFlash = true;
          //ghettoFlash();
        } 
      } 
      
      toggleHatch = currentEjector;//intake.getEjectorBoolean();
      
      intake.setKeepEjected(true);
    } else {
      wantedIntakeState.eject = false;
      toggleHatch = false;
      intake.setKeepEjected(false);
    }

    ghettoResetHatch();

    if(co.getBumperPressed(Hand.kLeft)) {
      //toggleIntake = !toggleIntake;
      wantedIntakeState.extend = !intake.getState().extend;
      
    } else {
      wantedIntakeState.extend = intake.getState().extend;
    }

    if(co.getXButtonPressed()) {
      
      wantedIntakeState.barOut = !intake.getState().barOut;
      
    } else {
      wantedIntakeState.barOut = intake.getState().barOut;
    }


    //System.out.println(intake.intakeValue());// kForward is up

    //wantedIntakeState.extend = toggleIntake;

     
    double elevatorSignal = 0.0;

    if(co.getStickButton(Hand.kRight)) {
      elevatorSignal = -0.5 * co.getY(Hand.kLeft);
      superstructure.setOpenLoopPower(elevatorSignal);
    } else {
      if(co.getBButtonPressed()) {
        superstructure.setDesiredHeight(midLevel);//Constants.MID); //25.0
        wantedIntakeState.extend = false;
        
      } else if(co.getAButtonPressed()) {
        superstructure.setDesiredHeight(Constants.LOW);
        
      } else if(co.getYButtonPressed()) {
        superstructure.setDesiredHeight(Constants.HIGH);
        wantedIntakeState.extend = false;
      } //Human Player station height for ball is 13.0 considering X to do that.

      if(co.getStartButton()) {
        elevatorSignal = -0.5 * co.getY(Hand.kLeft);
        
        //superstructure.setOpenLoopPower(elevatorSignal);
        superstructure.setDesiredHeight(elevator.getGoal() + elevatorSignal);
      }

    }
    
  
    intake.setWantedState(wantedIntakeState);


    if(intake.getBanner() != ballDetected) {
      if(ballDetected == false) {
        ballDetected = true;
        beginFlash = true;
        System.out.println(beginFlash);
      } else {
        ballDetected = false;
      }
    }    
    ghettoFlash();
    acquireTarget();
    
    if(!toggleGoGoGadget) {
      if(baseLeftTrigger.get()) {
        throttle *= 0.6;
        turn *= .8;
      }
    }
    

    
    signal = arcadeDriveHelper.arcadeDrive(throttle, turn);
    //!isHighGear if defaulting to high gear
    if(!isHighGear || !Util.epsilonEquals(throttle, 0.0, 0.1) || !Util.epsilonEquals(turn, 0.0, 0.1)) {
      autoDrive = false;
      driveSequence = AutoDriveSequence.PROCESS_ANGLE;
    }

    if(autoGadget) {
      handleAutomatedGadget();
    } else if(autoDrive) {
       //handleAutomatedDrive();
      //handleTestDrive();
    } else {
      drive.setOpenLoop(signal);
    }
    

    drive.setHighGear(isHighGear);
    
  }

  private void handleAutomatedDrive() {
    if(autoDrive) {
      System.out.println(driveSequence);
      switch(driveSequence) {
          //case ADJUST_ANGLE:
            /*if(!Util.epsilonEquals(processor.getTX(), 0.0, 5.0)) { //degrees
              if(drive.getDriveState() != DriveState.TURN_IN_PLACE) {
                drive.setDriveState(DriveState.TURN_IN_PLACE);
              }

            } else {
              drive.setOpenLoop(DriveSignal.NEUTRAL);
              driveSequence = AutoDriveSequence.DRIVE_TO_TARGET;
            }*/
            /*drive.startPathFollowing();
            driveSequence = AutoDriveSequence.DRIVE_TO_TARGET;
            break;*/
          /*case DRIVE_TO_TARGET:
            /*double currentDist;
            double tolerance;
            if(ballAutoDrive) {
              currentDist = processor.getDist();
              tolerance = 36.0;
            } else {
              tolerance = 30.0;
              currentDist = processor.getDist();
            }
            
            if(!Util.epsilonEquals(currentDist, 3.0, tolerance)) { //inches
              drive.setOpenLoop(new DriveSignal(-0.45, -0.45));
            } else {
              drive.setOpenLoop(DriveSignal.NEUTRAL);
              driveSequence = AutoDriveSequence.ADJUST_TO_SKEW;
            }*/
            /*if(drive.pathIsFinished()) {
              driveSequence = AutoDriveSequence.FINISH;
            }

            break;*/
          /*case ADJUST_TO_SKEW:
            if(!Util.epsilonEquals(processor.getTS(), 0.0, 5.0)) { //degrees
              if(drive.getDriveState() != DriveState.TURN_IN_PLACE) {
                drive.setRelativeAngle(processor.getTS());
                drive.setDriveState(DriveState.TURN_IN_PLACE);
              }
            } else {
              drive.setOpenLoop(DriveSignal.NEUTRAL);
              driveSequence = AutoDriveSequence.FINISH;
            }
            break;*/
          case PROCESS_ANGLE:
            if(Timer.getFPGATimestamp() - autoDriveTimestamp < 1.0) {
              if(Timer.getFPGATimestamp() - autoDriveTimestamp >= 0.5) {
                double yaw = processor.getYaw();
                data.yawAbs = Math.abs(yaw);
                data.txAbs = Math.abs(processor.getTX());
                data.sign = -Math.signum(yaw);
                data.x = 1.0*Math.abs(processor.getX()) / 12.0;
                data.y =  0.6* Math.abs(processor.getZ()) / 12.0;
                data.a1 = -data.sign * (90 - data.txAbs - data.yawAbs);
                data.a2 = data.sign * 90.0;
                data.angle = data.sign * ( data.txAbs + data.yawAbs);
                data.startHeading = drive.getHeading();
                //TODO: Maybe need to add startHeading to the angles of the paths
                data.path1 = Arrays.asList(new Waypoint(0.0, 0.0, Math.toRadians(data.startHeading + data.a1)), new Waypoint(data.x * Math.cos(Math.toRadians(data.startHeading + data.a1)),data.x * Math.sin(Math.toRadians(data.startHeading + data.a1)),Math.toRadians(data.startHeading + data.a1)));
                data.path2 = Arrays.asList(new Waypoint(0.0, 0.0, Math.toRadians(data.startHeading + data.angle)), new Waypoint(data.y * Math.cos(Math.toRadians(data.startHeading + data.angle)),data.y * Math.sin(Math.toRadians(data.startHeading + data.angle)),Math.toRadians(data.startHeading + data.angle)));
              }
            } else {
              driveSequence = AutoDriveSequence.TURN_HORIZONTAL;
              if(Math.abs(data.x) >= 15.0) {
                Trajectory trajectory = TrajectoryGenerator.generateQuinticHermiteSpline(drive.getConfig(), data.path1);
                AutoTrajectory traj = TrajectoryGenerator.makeLeftRightTrajectories(trajectory, Constants.WHEEL_BASE);
                data.traj1 = traj;
                Trajectory trajectory2 = TrajectoryGenerator.generateQuinticHermiteSpline(drive.getConfig(), data.path2);
                AutoTrajectory traj2 = TrajectoryGenerator.makeLeftRightTrajectories(trajectory2, Constants.WHEEL_BASE);
                data.traj2 = traj2;
                drive.setRelativeAngle(data.a1);
                
              } else {
                drive.setRelativeAngle(data.sign * data.txAbs);
              }
              
            }
            break;
          case TURN_HORIZONTAL:
            if(Math.abs(data.x) >= 15.0) {
              if(!Util.epsilonEquals(drive.getHeading(), data.startHeading + data.a1 , 5.0)) { //degrees
                if(drive.getDriveState() != DriveState.TURN_IN_PLACE) {
                  drive.setDriveState(DriveState.TURN_IN_PLACE);
                }
  
              } else {
                drive.setOpenLoop(DriveSignal.NEUTRAL);
                driveSequence = AutoDriveSequence.DRIVE_HORIZONTAL;
                drive.setTrajectory(data.traj1, false);
                drive.startPathFollowing();
              }
            } else {
              if(!Util.epsilonEquals(drive.getHeading(), data.startHeading + data.sign * data.txAbs , 5.0)) { //degrees
                if(drive.getDriveState() != DriveState.TURN_IN_PLACE) {
                  drive.setDriveState(DriveState.TURN_IN_PLACE);
                }
  
              } else {
                drive.setOpenLoop(DriveSignal.NEUTRAL);
                driveSequence = AutoDriveSequence.DRIVE_FORWARD;
                drive.setTrajectory(data.traj2, false);
                drive.startPathFollowing();
              }
            }
            break;
          case DRIVE_HORIZONTAL:
            if(drive.pathIsFinished()) {
              drive.setOpenLoop(DriveSignal.NEUTRAL);
              drive.setRelativeAngle(data.a2);
            }
            break;
          case TURN_VERTICAL:
            if(!Util.epsilonEquals(drive.getHeading(), data.startHeading + data.a2 , 5.0)) { //degrees
              if(drive.getDriveState() != DriveState.TURN_IN_PLACE) {
                drive.setDriveState(DriveState.TURN_IN_PLACE);
              }

            } else {
              drive.setOpenLoop(DriveSignal.NEUTRAL);
              driveSequence = AutoDriveSequence.DRIVE_FORWARD;
              drive.setTrajectory(data.traj2, false);
              drive.startPathFollowing();
            }
            break;
          case DRIVE_FORWARD:
            if(drive.pathIsFinished()) {
              driveSequence = AutoDriveSequence.FINISH;
            }
            break;
          case FINISH:
            drive.setOpenLoop(DriveSignal.NEUTRAL);
            autoDrive = false;
            break;
          default:
            autoDrive = false;
            
      }
    }
    
  }

  private void handleTestDrive() {
    
    Trajectory trajectory = TrajectoryGenerator.generateQuinticHermiteSpline(drive.getConfig(), Arrays.asList(new Waypoint(0.0, 0.0, 0.0), new Waypoint(6.0 * 12.0, 0.0, 0.0)));
    AutoTrajectory traj = TrajectoryGenerator.makeLeftRightTrajectories(trajectory, Constants.WHEEL_BASE);
    drive.setTrajectory(traj, false);
    drive.startPathFollowing();
    
      //System.out.println(autoDrive);
      //drive.setOpenLoop(new DriveSignal(-0.3, -0.3));
    

  }

  private void handleTurn() {
    if(autoDrive) {
      if(!Util.epsilonEquals(processor.getTX(), 0.0, 5.0)) { //degrees
        if(drive.getDriveState() != DriveState.TURN_IN_PLACE) {
          drive.setDriveState(DriveState.TURN_IN_PLACE);
          System.out.println("turning");
        }

      } else {
        drive.setOpenLoop(DriveSignal.NEUTRAL);
        autoDrive = false;
        
      }
    }
  }

  private void handleAutomatedGadget() {
    if(autoGadget) {
      System.out.println(gadgetAutoStep);
      switch(gadgetAutoStep) {
        
        /*case DRIVE:
          if(!gadget.getProximity()) {
            drive.setOpenLoop(new DriveSignal(-0.3, -0.3));
          } else {
            gadgetAutoStep = GadgetSequence.RAISE_BOTH;
          }
          break;*/
        case RAISE_BOTH:
          if(level2) {
            if(Math.abs(gadget.getObservedState().frontActuatorLength - Constants.LEVEL_2_LENGTH) >= 0.3 ) {
              drive.setOpenLoop(DriveSignal.NEUTRAL);
              gadget.setWantedState(GadgetWantedState.SYNCED);
              gadget.setFrontGoal(Constants.LEVEL_2_LENGTH);
              gadget.setRearGoal(Constants.LEVEL_2_LENGTH);
            } else {
              autoGadgetTimestamp = Timer.getFPGATimestamp();
              gadgetAutoStep = GadgetSequence.WAIT_BEFORE_DRIVE_FORWARD;
            }
          } else {
            if(Math.abs(gadget.getObservedState().frontActuatorLength - Constants.MAX_LENGTH) >= 0.5 ) {
              drive.setOpenLoop(DriveSignal.NEUTRAL);
              gadget.setWantedState(GadgetWantedState.SYNCED);
              gadget.setFrontGoal(Constants.MAX_LENGTH);
              gadget.setRearGoal(Constants.MAX_LENGTH);
            } else {
              autoGadgetTimestamp = Timer.getFPGATimestamp();
              gadgetAutoStep = GadgetSequence.WAIT_BEFORE_DRIVE_FORWARD;
            }
          }
          break;
        case WAIT_BEFORE_DRIVE_FORWARD:
          if(Timer.getFPGATimestamp() - autoGadgetTimestamp >= .3) {
            autoGadgetTimestamp = Timer.getFPGATimestamp();
            drive.setBrakeMode(true);
            gadgetAutoStep = GadgetSequence.DRIVE_ON;
          }
          break;
        case DRIVE_ON:
          if(Timer.getFPGATimestamp() - autoGadgetTimestamp < 4.0) {
            drive.setOpenLoop(DriveSignal.NEUTRAL);
            gadget.setGadgetDrive(1.0);
          } else {
            autoGadgetTimestamp = Timer.getFPGATimestamp();
            intake.getWantedState().extend = true;
            gadget.setGadgetDrive(0.0);
            gadgetAutoStep = GadgetSequence.WAIT_BEFORE_ADJUST;
          }
          break;
        case WAIT_BEFORE_ADJUST:
          if(Timer.getFPGATimestamp() - autoGadgetTimestamp >= 1.2) {
            gadgetAutoStep = GadgetSequence.ADJUST;
            autoGadgetTimestamp = Timer.getFPGATimestamp();
          } 
          break;
        case ADJUST:
          if(Timer.getFPGATimestamp() - autoGadgetTimestamp >= 0.5) {
            if(Math.abs(gadget.getObservedState().frontActuatorLength - Constants.MIN_LENGTH) >= 0.5 ) {
              drive.setOpenLoop(DriveSignal.NEUTRAL);
              gadget.setWantedState(GadgetWantedState.UNSYNCED);
              gadget.setFrontGoal(Constants.MIN_LENGTH);
              
            } else {
              autoGadgetTimestamp = Timer.getFPGATimestamp();
              gadgetAutoStep = GadgetSequence.DRIVE_FORWARD;
              
            }
          }
          break;
        case DRIVE_FORWARD:
          if(Timer.getFPGATimestamp() - autoGadgetTimestamp < 1.5) { //.7
            drive.setOpenLoop(new DriveSignal(-0.2, -0.2));
            gadget.setGadgetDrive(0.0);
          } else {
            autoGadgetTimestamp = Timer.getFPGATimestamp();
            gadgetAutoStep = GadgetSequence.WAIT_BEFORE_RAISE_REAR;
          }
          break;
        case WAIT_BEFORE_RAISE_REAR:
          if(Timer.getFPGATimestamp() - autoGadgetTimestamp < .9) {
            drive.setOpenLoop(new DriveSignal(-0.1, -0.1));
          } else {
            drive.setOpenLoop(DriveSignal.NEUTRAL);
            autoGadgetTimestamp = Timer.getFPGATimestamp();
            gadgetAutoStep = GadgetSequence.RAISE_REAR;
          }
          break;
        case RAISE_REAR:
          if(Math.abs(gadget.getObservedState().rearActuatorLength - Constants.MIN_LENGTH) >= 0.5 ) {
            drive.setOpenLoop(DriveSignal.NEUTRAL);
            gadget.setWantedState(GadgetWantedState.UNSYNCED);
            gadget.setFrontGoal(Constants.MIN_LENGTH);
            gadget.setRearGoal(Constants.MIN_LENGTH);
          } else {
            autoGadgetTimestamp = Timer.getFPGATimestamp();
            gadgetAutoStep = GadgetSequence.WAIT_BEFORE_FINISH;
          }
          break;
        case WAIT_BEFORE_FINISH:
          if(Timer.getFPGATimestamp() - autoGadgetTimestamp >= .2) {
            autoGadgetTimestamp = Timer.getFPGATimestamp();
            gadgetAutoStep = GadgetSequence.FINISH;
          }
          break;
        case FINISH:
          if(Timer.getFPGATimestamp() - autoGadgetTimestamp < 2.0) {
            drive.setOpenLoop(new DriveSignal(-0.2, -0.2));
          } else {
            drive.setOpenLoop(DriveSignal.NEUTRAL);
            gadgetAutoStep = GadgetSequence.OFF;
            autoGadget = false;
            intake.getWantedState().extend = false;
          }
          break;
        default:
          autoGadget = false;
          break;
      }
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    try {
      ///operatorControls();

      //setStickAndWheel(SmartDashboard.getBoolean("Stick and Wheel", false));

      boolean highGear =  !stick.getLowGear();
      
      DriveSignal signal = cheesyDriveHelper.cheesyDrive(stick.getThrottle(), wheel.getWheel(), stick.getQuickTurn(), highGear);

      drive.setHighGear(highGear);
      drive.setOpenLoop(signal);
			
		} catch(Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
    }
    
    allPeriodic();
  }

  @Override
	public void disabledInit() {
		try {
      CrashTracker.logDisabledInit();
      
      //base.getPOV();
			
			if(autoModeExecuter != null) {
				autoModeExecuter.stop();
			}
			
			autoModeExecuter = null;
			
			internalLooper.stop();
			
			subsystemManager.stop();
			
      drive.setOpenLoop(DriveSignal.NEUTRAL);

      slewRateLimiter.reset();
      turnLimiter.reset();
      
      drive.setBrakeMode(false);
			
		} catch(Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
  }

  

  public void ghettoResetBase() {
    if(toggleGoGoGadget) {

      if(ghettoReset == 0) {
        base.setRumble(RumbleType.kLeftRumble, 1.0);
        base.setRumble(RumbleType.kRightRumble, 1.0);
      }
      ghettoReset++;
      if(ghettoReset >= 20) {
        base.setRumble(RumbleType.kLeftRumble, 0.0);
        base.setRumble(RumbleType.kRightRumble, 0.0);
        ghettoReset = 0;
      }
    } else {
      if(!toggleHatch) {
        base.setRumble(RumbleType.kLeftRumble, 0.0);
        base.setRumble(RumbleType.kRightRumble, 0.0);
        ghettoReset = 0;
      }
    }
  }

  public void ghettoFlash() {
    if(beginFlash) {
      if(LEDCounter == 0) {
        processor.setLedMode(2);
      }
      LEDCounter++;
      if(LEDCounter >= 30) {
        LEDCounter = 0;
        beginFlash = false;
        processor.setLedMode(1);
      }
    }
  }

  public void acquireTarget() {
    if(beginAcquireTarget) {
      if(targetCounter == 0) {
        processor.setLedMode(3);
        //processor.setWebCamMode(0); 0 is processor mode
      }
      targetCounter++;
      if(targetCounter >= 200) {
        targetCounter = 0;
        beginAcquireTarget = false;
        processor.setLedMode(1);
        //processor.setWebCamMode(1);
      }
    }
  }

  public void ghettoResetHatch() {
    if(toggleHatch) {

      if(ghettoReset1 == 0) {
        base.setRumble(RumbleType.kLeftRumble, 1.0);
        base.setRumble(RumbleType.kRightRumble, 1.0);
        co.setRumble(RumbleType.kLeftRumble, 1.0);
        co.setRumble(RumbleType.kRightRumble, 1.0);
      }
      ghettoReset1++;
      if(ghettoReset1 >= 20) {
        base.setRumble(RumbleType.kLeftRumble, 0.0);
        base.setRumble(RumbleType.kRightRumble, 0.0);
        co.setRumble(RumbleType.kLeftRumble, 0.0);
        co.setRumble(RumbleType.kRightRumble, 0.0);
        ghettoReset1 = 0;
      }
    } else {
      
      co.setRumble(RumbleType.kLeftRumble, 0.0);
      co.setRumble(RumbleType.kRightRumble, 0.0);
      ghettoReset1 = 0;
    }
  }
  
  @Override
	public void disabledPeriodic() {
    //code for setting the auto mode in here
    if(co.getAButton()) {
      autoMode = AutoMode.ROCKET_SHIP;
    } else if(co.getStartButtonPressed()) {
      autoMode = AutoMode.HUMAN_PLAYER;
    } /*else if(base.getXButtonPressed()) {
      autoMode = AutoMode.CARGO_LEFT_PANEL_MODE;
    } else if(base.getBButtonPressed()) {
      autoMode = AutoMode.CARGO_RIGHT_PANEL_MODE;
    } else if(base.getRawButtonPressed(9)) {
      autoMode = AutoMode.CARGO_LEFT_PANEL_LEVEL_2_MODE;
    } else if(base.getRawButtonPressed(10)) {
      autoMode = AutoMode.CARGO_RIGHT_PANEL_LEVEL_2_MODE;
    }*/
   /*if(base.getAButtonPressed()) {
     if(toggleHatch) {
       toggleHatch = !toggleHatch;
     } else {
       toggleHatch = !toggleHatch;
     }
   }*/
		
		System.out.println(autoMode);
		
		allPeriodic();
	}

  public void allPeriodic() {
    //Do we want to monitor connection to the robot?
		//ConnectionMonitor.getInstance().setLastPacketTime(Timer.getFPGATimestamp());
    //SmartDashboard.putNumber("Sanic", sanic.getVoltage() * 12.0 / 2.06 + 2.0 - 0.3);
    
    //SmartDashboard.putBoolean("Currently Using Stick?", isStickAndWheel);
    /*NetworkTableEntry targetOffsetAngle_Horizontal = table.getEntry("tx");
    NetworkTableEntry targetOffsetAngle_Vertical = table.getEntry("ty");
    NetworkTableEntry targetSkew = table.getEntry("ts");
    NetworkTableEntry targetValid = table.getEntry("tv");
    
    t_x = targetOffsetAngle_Horizontal.getDouble(0.0);
    t_y = targetOffsetAngle_Vertical.getDouble(0.0);
    t_s = targetSkew.getDouble(0.0);
    t_v = targetValid.getDouble(0.0);
    dist = calculateDist(t_y);

    SmartDashboard.putNumber("t_x", t_x);
    SmartDashboard.putNumber("skew", t_s); //Gives direction you must turn to face goal
    SmartDashboard.putNumber("valid", t_v);
    SmartDashboard.putNumber("Hatch Distance", dist);    
    SmartDashboard.putNumber("Ball Distance", ballDist);*/
		subsystemManager.outputToSmartDashboard();
    subsystemManager.writeToLog();
    
    internalLooper.outputToSmartDashboard();
  }
  
  private double calculateDist(double vert_angle) {
    double a2 = vert_angle;
    return (Constants.h2 - Constants.h1) / Math.tan(Math.toRadians(Constants.a1 + a2));

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
