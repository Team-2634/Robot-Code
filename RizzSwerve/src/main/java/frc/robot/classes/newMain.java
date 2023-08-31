package frc.robot.classes;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SPI;

public class newMain extends TimedRobot {
    //Global variables - Do not overuse!
    //Robot Components
    SwerveDriveTrain driveTrain;

    //Game Timers
    Timer timerRobot = new Timer();
    Timer timerAuto = new Timer();

    //Hardware Imports
    AHRS navx = new AHRS(SPI.Port.kMXP);

    final XboxController driving_xBoxCont = new XboxController(0);
    final XboxController arm_xBoxCont = new XboxController(1);


    //TODO move offsets to config?
    //TODO write explaination of what this is?
    public static final Map<MotorLocation, Translation2d> motorLocations = initMotorLocations();
    //This lets you initialze a static hashmap at runtime
    private static Map<MotorLocation, Translation2d> initMotorLocations() { 
        HashMap<MotorLocation, Translation2d> motorLocationsInit = new HashMap<>();
        motorLocationsInit.put(MotorLocation.FrontLeft_Drive, new Translation2d(0.340, 0.285));
        motorLocationsInit.put(MotorLocation.FrontRight_Drive, new Translation2d(0.340, -0.285));
        motorLocationsInit.put(MotorLocation.BackLeft_Drive, new Translation2d(-0.340, 0.285));
        motorLocationsInit.put(MotorLocation.BackRight_Drive, new Translation2d(-0.340, -0.285));
        return Collections.unmodifiableMap(motorLocationsInit);
    }

    /*
    public static final Map<MotorType, Object> motorTypes = initMotorTypes();
    private static Map<MotorType, Object> initMotorTypes() { 
        HashMap<MotorType, Object> motorTypesInit = new HashMap<>();
        motorTypesInit.put(MotorType.WPI_TalonFX, new WPI_TalonFX(1));
        //motorTypesInit.put(MotorType.CANSparkMax, CANSparkMax);
        return Collections.unmodifiableMap(motorTypesInit);
    }
    //TODO make use of, remove, fix value...
    */


    //TODO make this class Robot, called by main.java.  This is the entrypoint
    public newMain(){
       
        Motor[] motors = {
            //TODO move offset values to config file?
            new Motor<WPI_TalonFX>(new WPI_TalonFX(7), MotorModel.WPI_TalonFX, MotorLocation.FrontLeft_Drive),
            new Motor<WPI_TalonFX>(new WPI_TalonFX(1), MotorModel.WPI_TalonFX, MotorLocation.FrontRight_Drive),
            new Motor<WPI_TalonFX>(new WPI_TalonFX(5), MotorModel.WPI_TalonFX, MotorLocation.BackLeft_Drive),
            new Motor<WPI_TalonFX>(new WPI_TalonFX(3), MotorModel.WPI_TalonFX, MotorLocation.BackRight_Drive),

            new Motor<WPI_TalonFX>(new WPI_TalonFX(6), MotorModel.WPI_TalonFX, MotorLocation.FrontLeft_Steer, new WPI_CANCoder(3),  197.19),
            new Motor<WPI_TalonFX>(new WPI_TalonFX(0), MotorModel.WPI_TalonFX, MotorLocation.FrontRight_Steer, new WPI_CANCoder(0),  31.8),
            new Motor<WPI_TalonFX>(new WPI_TalonFX(4), MotorModel.WPI_TalonFX, MotorLocation.BackLeft_Steer, new WPI_CANCoder(2),  285.205),
            new Motor<WPI_TalonFX>(new WPI_TalonFX(2), MotorModel.WPI_TalonFX, MotorLocation.BackRight_Steer, new WPI_CANCoder(1),  50.4)
        };

        this.driveTrain = new SwerveDriveTrain(motors, Config.teleopSteer_P, Config.teleopSteer_I, Config.teleopSteer_D);
    }


    // execution Functions vvvvv
    @Override
    public void robotInit() {
        timerRobot.reset();
        timerRobot.start();

        UsbCamera camera0 = CameraServer.startAutomaticCapture(0); //TODO these shouldnt be defined here
        UsbCamera camera1 = CameraServer.startAutomaticCapture(1);

        driveTrain.setMotorBrakes();
        //arm.setMotorBrakes();

        driveTrain.setInverted(true);
        driveTrain.setContinouousInput();

        navx.calibrate();
        navx.reset();

        driveTrain.resetEncoders();
        //arm.resetEncoders();

        driveTrain.resetPIDs();
        driveTrain.setDriveEncoderPosition(0);
    }

    @Override
    public void robotPeriodic() {
        // encoder drive variables vvv
        double frontLeftDriveDisplacement_Meters = driveTrain.getDriveEncoderPosition(MotorLocation.FrontLeft_Drive) * globalFunctions.getEncoderTicksMPS();
        globalFunctions.toDashboard("Drive_Distance_Meters x axis: ", frontLeftDriveDisplacement_Meters);

        double frontRightDriveDisplacement_Meters = driveTrain.getDriveEncoderPosition(MotorLocation.FrontRight_Drive) * globalFunctions.getEncoderTicksMPS();
        globalFunctions.toDashboard("Drive_Distance_Meters yaxis: ", frontRightDriveDisplacement_Meters);

        double frontLeftSteer_Rad = driveTrain.getDriveEncoderPosition(MotorLocation.FrontLeft_Steer) * globalFunctions.getEncoderTicksMPS_steer();
        globalFunctions.toDashboard("Drive_Rotation_Radians: ", frontLeftSteer_Rad);

        // navX2 vvv
        //TODO, clean up and un-duplicate
        globalFunctions.toDashboard("Yaw_Deg", navx.getYaw());
        globalFunctions.toDashboard("navxYaw_Rad ", Units.degreesToRadians(navx.getYaw()));
        globalFunctions.toDashboard("angleRad: ", Math.toRadians(navx.getAngle()));
        globalFunctions.toDashboard("Pitch_deg", navx.getPitch());
        globalFunctions.toDashboard("Roll_Deg", navx.getRoll());
        globalFunctions.toDashboard(navx);

        // encoders vvv
        globalFunctions.toDashboard("frontLeftAbs Offset", driveTrain.getAbsoluteEncoderPosition(MotorLocation.FrontLeft_Steer));
        globalFunctions.toDashboard("frontRightAbs Offset", driveTrain.getAbsoluteEncoderPosition(MotorLocation.FrontRight_Steer));
        globalFunctions.toDashboard("backLeftAbs Offset", driveTrain.getAbsoluteEncoderPosition(MotorLocation.BackLeft_Steer));
        globalFunctions.toDashboard("backRightAbs Offset", driveTrain.getAbsoluteEncoderPosition(MotorLocation.BackRight_Steer));

        // pid charts vvv
        // auto vvv
        //toDashboard("drive PID (atm used for x, y and turn of autonomous)", drive.getPositionError());
        //toDashboard("PID_armAngle", PID_armAngle.getPositionError());

        // aline vvv
        //TODO
        //toDashboard("pid_frontLeft_Error", pidFrontLeftTurn.getPositionError());
        //toDashboard("pid_frontRight_Error", pidFrontRightTurn.getPositionError());
        //toDashboard("pid_backLeft_Error", pidBackLeftTurn.getPositionError());
        //toDashboard("pid_backRight_Error", pidBackRightTurn.getPositionError());
    }

    @Override
    public void autonomousInit() {
        timerAuto.reset();
        timerAuto.start();

        driveTrain.setPIDs(Config.autoSteer_P, Config.autoSteer_I, Config.autoSteer_D);

        //dSolenoidClaw.set(Value.kReverse);
    }

    @Override
    public void autonomousPeriodic() {
        //TODO Call new actions handler, pick which auto set to run
        // autoTopCone(); //balance
        //autoTopAndBottom(); //not balance
    } 

    @Override
    public void teleopInit() {
        timerAuto.reset();
        timerAuto.start();

        driveTrain.setPIDs(Config.teleopSteer_P, Config.teleopSteer_I, Config.teleopSteer_D);

        //dSolenoidClaw.set(Value.kReverse);
    }

    @Override
    public void teleopPeriodic() {

        //limitationArmRise(armRad_current);
        //toDashboard("armAngleLimited: ", armAngleLimited);

        //limitationArmExtend(extenstionEncoder_CurrentMetres);
        // toDashboard("armExtendLimited: ", armExtendLimited);
        // setMotorSpeed_limitinExtendWHenToFar(-armTalonExtenstionSpeed_autoRetreat);

        //toDashboard("retractLimitSwitch: ", extendLimitSwitch.get());
        //System.out.println(extendLimitSwitch.get());


        
        //// Swerve vvv
        //Get scaled Axis inputs from controller
        double contXSpeed = globalFunctions.removeDeadzone(this.driving_xBoxCont, 1) * Config.XdriveSensitivity;
        double contYSpeed = globalFunctions.removeDeadzone(this.driving_xBoxCont, 0) * Config.YdriveSensitivity;
        double contTurnSpeed = globalFunctions.removeDeadzone(this.driving_xBoxCont, 4) * Config.turningSensitivity;
 
        if (Config.useFieldOrientation) {
            double botYawAngle_Rad = Math.toRadians(navx.getAngle());
            contXSpeed = contXSpeed * Math.cos(botYawAngle_Rad) - contYSpeed * Math.sin(botYawAngle_Rad);
            contYSpeed = contXSpeed * Math.sin(botYawAngle_Rad) + contYSpeed * Math.cos(botYawAngle_Rad);
        }
        this.driveTrain.GO(contXSpeed, contYSpeed, contTurnSpeed);

        // Button inputs: //TODO cleanup, set up blanks, controlSchemes, use better namings
        // robot arm
        double armDown = arm_xBoxCont.getRightTriggerAxis();
        double armUp = arm_xBoxCont.getLeftTriggerAxis();
        boolean claw_xBox = arm_xBoxCont.getBButtonPressed();
        boolean extendArm = arm_xBoxCont.getLeftBumper();
        boolean retractArm = arm_xBoxCont.getRightBumper();
        boolean expel = arm_xBoxCont.getAButton();
        boolean intake = arm_xBoxCont.getXButton();
        boolean clawIntake_and_Extend = arm_xBoxCont.getYButton();
        boolean slowClawWheels = driving_xBoxCont.getRightBumper();

        //TODO
        //robotArm(armDown, armUp, claw_xBox, extendArm, retractArm, expel, intake, clawIntake_and_Extend, slowClawWheels);
    }
}
