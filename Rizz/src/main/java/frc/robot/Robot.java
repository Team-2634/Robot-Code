package frc.robot;

//import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Elevator;
import frc.robot.systems.Arm;
import frc.robot.systems.LimeLight;
//import edu.wpi.first.wpilibj.SPI;

public class Robot extends TimedRobot {

    Driver driver = new Driver();
    Climber climber = new Climber();
    Timer matchTimer = new Timer();
    Elevator elevator = new Elevator();
    LimeLight limelight = new LimeLight();
    Arm arm = new Arm();

    AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI); 

    Auto auto = new Auto(driver, climber, navx, matchTimer, elevator, arm, limelight);
    Teleop teleop = new Teleop(driver, climber, navx, elevator, arm, limelight);

    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    public Robot(){
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("My Auto", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);
    }

    @Override
    public void robotInit() {

        matchTimer.reset();
        matchTimer.start();
        navx.reset();
        driver.initialize(); // navx also reset after pose
        elevator.elevatorInitiallize();
        arm.armInitiallize();
        climber.initializeClimb();

    }
    
    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("X-offset", limelight.Xoffset());
        SmartDashboard.putNumber("Y-offset", limelight.Yoffset());
        SmartDashboard.putNumber("AprilTag Yaw",limelight.targetYaw());
        SmartDashboard.putNumber("AprilTag tx",limelight.limelightTestValues(1));
        SmartDashboard.putNumber("AprilTag ty",limelight.limelightTestValues(2));
        SmartDashboard.putNumber("AprilTag tz",limelight.limelightTestValues(3));

        driver.updatePose();
    }
    
    @Override
    public void autonomousInit() {
        auto.restartTimer();

        m_autoSelected = m_chooser.getSelected();
        System.out.println("Auto Selected: " + m_autoSelected);
    }
    
    @Override
    public void autonomousPeriodic() {
        auto.autoDriveByDistanceTest();
        }

    
    @Override
    public void teleopInit() {
    }
    
    @Override
    public void teleopPeriodic() {
        teleop.drive();
        teleop.elevator();
        teleop.clamp();
        teleop.elevatorArm();
        // teleop.climber();
        //teleop.driveByDistanceTest();
        teleop.arm();

        teleop.limelightFunctions();
        SmartDashboard.putNumber("Pose x-value", driver.getPose().getX());
        SmartDashboard.putNumber("Pose x-value", driver.getPose().getY());
        SmartDashboard.putNumber("arm angle in rad", arm.getArmAngleRad());
        SmartDashboard.putNumber("elevator height", elevator.getElevatorHeight());
        SmartDashboard.putNumber("KP", Constants.kpAuto);
        SmartDashboard.putNumber("KI", Constants.kiAuto);
        SmartDashboard.putNumber("KD", Constants.kdAuto);
        SmartDashboard.putNumber("Test test", 69); //add the box to smart dashboard
        testnum = SmartDashboard.getNumber("Test test", 0);
        //Testing Changing of PID values
        System.out.println(testnum); //if prints out 69 we can change out values

    }

    double testnum;

}