package frc.robot;

import org.ejml.data.ElementLocation;

//import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.AHRS;

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
        driver.initialize();
        elevator.elevatorInitiallize();
        arm.armInitiallize();
        climber.initializeClimb();

    }
    
    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("X-offset", limelight.Xoffset());
        SmartDashboard.putNumber("Y-offset", limelight.Yoffset());
    }
    
    @Override
    public void autonomousInit() {
        auto.restartTimer();

        m_autoSelected = m_chooser.getSelected();
        System.out.println("Auto Selected: " + m_autoSelected);
    }
    
    @Override
    public void autonomousPeriodic() {
        //auto.autoLeft();
        //Hi hans this is for testing
        auto.autoLeftBlueAlliance();
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
        //teleop.arm();
        teleop.armTest();

        teleop.limelightFunctions();
        SmartDashboard.putNumber("Elevator Height", elevator.getElevatorHeight());
        SmartDashboard.putNumber("Arm Angle in Rad", arm.getArmAngleRad());
    }
}
