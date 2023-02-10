// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.SPI;

public class Robot extends TimedRobot {

    private final Constants cont = new Constants();
    private final DifferentialDrive m_robotDrive = new DifferentialDrive(cont.m_leftSide, cont.m_rightSide);
    private final Timer timer = new Timer();
    private final XboxController xbox = new XboxController(0);
    private final DoubleSolenoid dSolenoidShifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    private final DoubleSolenoid dSolenoidCool = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    private final Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    private final double Scale = 250, offset = -25;
    private final AnalogPotentiometer potentiometer = new AnalogPotentiometer(0, Scale, offset);

    private AHRS navx;

    public Value coolingSolenoid;
    boolean xButtonPressed = false;
    double previousPitch = 0;

    public Robot() {
        m_robotDrive.isSafetyEnabled();
        try {
            navx = new AHRS(SPI.Port.kMXP);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }
    }

    @Override
    public void robotInit() {
        double currentPsi = potentiometer.get();
        int psiCap = 117;
        if (currentPsi <= psiCap) {
            compressor.enableDigital();
        } else if (currentPsi > 119) {
            compressor.disable();
        }
    }

    @Override
    public void autonomousInit() {
        navx.reset();
        timer.reset();
        timer.start();
    }

    public void setMotorsNeutral() {
        /*
         * leftFront.setNeutralMode(NeutralMode.Brake);
         * leftBack.setNeutralMode(NeutralMode.Brake);
         * rightFront.setNeutralMode(NeutralMode.Brake);
         * rightBack.setNeutralMode(NeutralMode.Brake);
         */

        cont.leftFront.setIdleMode(IdleMode.kBrake);
        cont.leftBack.setIdleMode(IdleMode.kBrake);
        cont.rightFront.setIdleMode(IdleMode.kBrake);
        cont.rightBack.setIdleMode(IdleMode.kBrake);
    }

    public void balanceRobot(double robotPitch) {
        
        RobotAngle currentPitchState = getPitchState(robotPitch);
        double fastLeanSpeed = 0.25;
        double slowLeanSpeed = 0.15;

        if (currentPitchState == RobotAngle.Balanced) {
            m_robotDrive.arcadeDrive(0, 0);

        } else if (currentPitchState == RobotAngle.Forward) {
            m_robotDrive.arcadeDrive(0, fastLeanSpeed);

        } else if (currentPitchState == RobotAngle.leaningForward) {
            if (robotPitch > previousPitch) {
                m_robotDrive.arcadeDrive(0, slowLeanSpeed);
            } else if (robotPitch <= previousPitch) {
                m_robotDrive.arcadeDrive(0, 0);
            }

        } else if (currentPitchState == RobotAngle.Backward) {
            m_robotDrive.arcadeDrive(0, -fastLeanSpeed);

        } else if (currentPitchState == RobotAngle.leaningBackward) {
            if (robotPitch < previousPitch) {
                m_robotDrive.arcadeDrive(0, -slowLeanSpeed);
            } else if (robotPitch >= previousPitch) {
                m_robotDrive.arcadeDrive(0, 0);
            }
        }
        previousPitch = robotPitch;
    }
/*
    @Override
    public void autonomousPeriodic() {
        double robotPitch = navx.getPitch();

        SmartDashboard.putData(navx);
        SmartDashboard.putNumber("NAVXANGLE", robotPitch);

        setMotorsNeutral();

        balanceRobot();
    }
 */
    enum RobotAngle {
        Forward,
        leaningForward,
        Balanced,
        leaningBackward,
        Backward,
        unset
    }

    double tolerance = 4;
    double tiltBack = 15;
    double tiltFwd = -12;
    double bal = 0;

    public RobotAngle getPitchState(double robotPitch) {
        RobotAngle result = RobotAngle.unset;

        if (Math.abs(robotPitch) < tolerance) {
            result = RobotAngle.Balanced;

        } else if (robotPitch > tiltBack) {
            result = RobotAngle.Backward;

        } else if (robotPitch < tiltBack && robotPitch > tolerance) {
            result = RobotAngle.leaningBackward;

        } else if (robotPitch > tiltFwd && robotPitch < -tolerance) {
            result = RobotAngle.leaningForward;

        } else if (robotPitch < tiltFwd) {
            result = RobotAngle.Forward;
        }
        SmartDashboard.putString("Robot Pitch", result.toString());
        return result;
    }

    @Override
    public void teleopInit() {
        timer.reset();
        timer.start();
        navx.reset();
        cont.m_leftSide.setInverted(false);
        cont.m_rightSide.setInverted(false);
    }
/* 
    private void pulsePiston(double teleopTime) {
        int pulseFreq = 15;
        int pulseDuration = 1;
        if (teleopTime % pulseFreq < pulseDuration) {
            dSolenoidCool.set(Value.kForward);
        } else {
            dSolenoidCool.set(Value.kReverse);
        }
    }
*/
    @Override
    public void teleopPeriodic() {
        double teleopTime = timer.get();

        // pulsePiston(teleopTime);

        double l_speed = cont.m_leftSide.get();
        double r_speed = cont.m_rightSide.get();
        double pitchNavx = navx.getPitch();
        SmartDashboard.putData(navx);
        SmartDashboard.putNumber("NAVXANGLE", pitchNavx);
        SmartDashboard.putNumber("Speed Left", l_speed);
        SmartDashboard.putNumber("SPeed Right", r_speed);

        setMotorsNeutral();

        m_robotDrive.arcadeDrive(xbox.getRawAxis(4) * 0.8, xbox.getRawAxis(1) * 0.8);

        if (xbox.getAButton() == true) {
            dSolenoidShifter.set(Value.kForward);
        } else if (xbox.getBButton() == true) {
            dSolenoidShifter.set(Value.kReverse);
        }
        /*
         * if (xbox.getRightBumper() == true) {
         * rightIntake.set(-1);
         * leftIntake.set(1);
         * } else if (xbox.getLeftBumper() == true) {
         * leftIntake.set(-1);
         * rightIntake.set(1);
         * } else {
         * leftIntake.set(0);
         * rightIntake.set(0);
         * }
         */

        if (xbox.getXButtonPressed() == true) {
            xButtonPressed = !xButtonPressed;
        }

        double deadzone = 0.5;
        if (xbox.getRawAxis(4) > deadzone ||
                xbox.getRawAxis(4) < -deadzone &&
                        xbox.getRawAxis(1) > deadzone
                ||
                xbox.getRawAxis(1) < -deadzone) {
            xButtonPressed = false;
        }

        if (xButtonPressed == true) {
            balanceRobot(pitchNavx);
        }
        SmartDashboard.putBoolean("Balance mode: ", xButtonPressed);
        if (xbox.getYButton() == true) {
            navx.reset();
        }
    }
}