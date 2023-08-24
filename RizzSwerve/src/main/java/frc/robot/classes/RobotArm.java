package frc.robot.classes;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class RobotArm {
    public Motor shoulderMotor;
    public Motor elbowMotor;
    public Motor wristMotor;
    public Motor handMotor;

    // TODO make extends motor
    public final WPI_TalonFX leftArmSide = new WPI_TalonFX(9);
    public final WPI_TalonFX rightArmSide = new WPI_TalonFX(8);
    private final DifferentialDrive shoulderMotorGroup = new DifferentialDrive(leftArmSide, rightArmSide);

    final WPI_TalonFX armExtenstion = new WPI_TalonFX(10);

    public Motor extension;

    // pnuematics vvv
    Boolean clawToggle = true;
    private final DoubleSolenoid dSolenoidClaw = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 4);

    private final CANSparkMax claw_Wheels = new CANSparkMax(13,
            com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless); // TODO fix input

    // PIDController pid;

    public RobotArm() {
        // these are for the arm lift vvv
        double liftArmSide_GearRatio = 64 * (60 / 15);
        double armRotate_ToRad = ((1.0 / liftArmSide_GearRatio) * 2 * Math.PI) / 2048;
        double armRad_current;
        double kp_armAngle = 0.5, ki_armAngle = 0.05, kd_armAngle = 0.05;
        final PIDController PID_armAngle = new PIDController(kp_armAngle, ki_armAngle, kd_armAngle);
        // flag to indicate if arm angle is being limited
        // private boolean armAngleLimited = false; TODO make get isLimited function

        // arm extend vvv
        // final WPI_TalonFX armTalonExtenstion = new WPI_TalonFX(10);
        // private boolean armExtendLimited = false;
        double armExtenstion_gearRatio = 1 / 36.0;
        double armTalonExtenstionSpeed_Out = 0.95; // 0.83
        double armTalonExtenstionSpeed_In = 0.95; // 0.83.
        double armTalonExtenstionSpeed_autoExtend = 0.20;
        double armTalonExtenstionSpeed_autoRetreat = 0.10;
        double armExtenstion_ToMetres = (armExtenstion_gearRatio * Math.PI * Units.inchesToMeters(2.75)) / 2048.0; // metres
        double extenstionEncoder_CurrentMetres;
        double kp_armE = 0.5, ki_armE = 0, kd_armD = 0;
        final PIDController pidArmExtensController = new PIDController(kpAuto, kiAuto, kdAuto);

        // claw_Wheels vvv

    }
    // TODO motor setters

    // TODO encoder functions
    // rightArmSide.setSelectedSensorPosition(0);

    // TODO Motor brakes
    // leftArmSide.setNeutralMode(NeutralMode.Brake);
    // rightArmSide.setNeutralMode(NeutralMode.Brake);
    // armTalonExtenstion.setNeutralMode(NeutralMode.Brake);
    // claw_Wheels.setIdleMode(IdleMode.kBrake);

    // TODO move to constructor
    double maxArmAngle_Rad = -1.95;
    double minArmAngle_Rad = 0.015;

    double maxArmExtend_Metres = 0.79;
    double minArmExtend_Metres = 0.005;

    // TODO split forw/rev speeds
    double armRotationSpeed = 0.75;

    double armExtensionSpeed_Out = 0.95; // 0.83
    double armExtensionSpeed_In = 0.95; // 0.83.
    // double armTalonExtenstionSpeed_autoExtend = 0.20;
    // double armTalonExtenstionSpeed_autoRetreat = 0.10;

    double clawIntakeSpeed_In = 1;
    double clawIntakeSpeed_Out = 0.40;
    double clawIntakeSpeed_In_SLOW = 0.10; //As a percentage of clawIntakeSpeed_In
    //

    // TODO pass in motor to use, pass in speed min/max obj. (limitor?)
    /**
     * @param controlInput percentage power between -1 and 1
     *
     * @return void
     */
    public void armRaise(double controlInput) {
        double inputTolerance = 0.1;

        double currentAngle = 0; // TODO

        if (controlInput >= inputTolerance) {
            double goInput = limitMovementArm(controlInput, currentAngle);
            double goSpeed = goInput * armRotationSpeed;

            goSpeed = globalFunctions.clamp(goSpeed, 1, -1); // todo move to own function
            shoulderMotorGroup.tankDrive(goSpeed, -goSpeed);

        } else if (controlInput <= -inputTolerance) {
            double goInput = limitMovementArm(controlInput, currentAngle);
            double goSpeed = goInput * armRotationSpeed;

            goSpeed = globalFunctions.clamp(goSpeed, 1, -1); // todo move to own function
            shoulderMotorGroup.tankDrive(-goSpeed, goSpeed);

        } else {
            shoulderMotorGroup.tankDrive(0, 0);

        }
    }

    /**
     * @param controlInput percentage power between -1 and 1
     *
     * @return void
     */
    public void armExtend(double controlInput) {
        double inputTolerance = 0.1;

        double currentExtend_Metres = 0; // TODO

        if (controlInput >= inputTolerance) {
            double goInput = limitMovementExtend(controlInput, currentExtend_Metres);
            double goSpeed = goInput * armExtensionSpeed_Out;

            goSpeed = globalFunctions.clamp(goSpeed, 1, -1); // todo move to own function
            armExtenstion.set(goSpeed);

        } else if (controlInput <= -inputTolerance) {
            double goInput = limitMovementExtend(controlInput, currentExtend_Metres);
            double goSpeed = goInput * armExtensionSpeed_In;

            goSpeed = globalFunctions.clamp(goSpeed, 1, -1); // todo move to own function
            armExtenstion.set(-goSpeed);
        } else {
            shoulderMotorGroup.tankDrive(0, 0);

        }
    }

    public double limitMovementArm(double controlInput, double currentAngle) {
        return limitMovement(controlInput, currentAngle, maxArmAngle_Rad, minArmAngle_Rad);
    }

    public double limitMovementExtend(double controlInput, double currentExtend) {
        return limitMovement(controlInput, currentExtend, maxArmExtend_Metres, minArmExtend_Metres);
    }

    public double limitMovement(double controlInput, double currentAngle, double max, double min) {
        // if going up && at +ve limit
        if (controlInput > 0 && currentAngle >= max) {
            return 0;
        }
        // if going down && at -ve limit
        else if (controlInput < 0 && currentAngle <= min) {
            return 0;
        } else {
            return controlInput;
        }
    }

    // TODO rework to make useful for logging
    public boolean isLimitMovement(double controlInput, double currentAngle) {
        // if going up && at +ve limit
        if (controlInput > 0 && currentAngle >= maxArmAngle_Rad) {
            return true;
        }
        // if going down && at -ve limit
        else if (controlInput < 0 && currentAngle <= minArmAngle_Rad) {
            return true;
        } else {
            return false;
        }
    }

    //TODO Move claw to own class
    public void clawActivate(boolean state, DoubleSolenoid solenoid) {
        if (state == true) {
            solenoid.set(Value.kForward);
        } else if (state == false) {
            solenoid.set(Value.kReverse);
        }
    }

    // double clawIntakeSpeed_In = -1;
    // double clawIntakeSpeed_Out = 0.40;
    // double clawIntakeSpeed_In_SLOW = -0.10;

    public void intake(double power, Motor wheels){
        if (power > 0) {
            wheels.GO(power * clawIntakeSpeed_In);
        } else if (power < 0) {
            wheels.GO(power * clawIntakeSpeed_Out);
        } else{
            wheels.GO(0);
        }
    }

    // TODO clean up inputs / break up function
    public void move(double armDown, double armUp, Boolean claw_button, Boolean extendArm, Boolean retractArm,
            boolean claw_expel, boolean claw_intake, boolean clawIntake_and_Extend, boolean slowClawWheels) {

        // arm angle rotaion vvv
        // Analog Input pair (two doubles/stick values)
        double armInputTotal = armUp - armDown;
        armRaise(armInputTotal);

        // arm extendo vvv
        // Digital Tri-state Input (two booleans/buttons, or neither)
        double extendInputTotal = (extendArm ? 1 : 0) + (retractArm ? -1 : 0); // TODO explain ternaray
        armExtend(extendInputTotal);

        // THE CLAW vvv
        // Toggle button
        if (claw_button == true) {
            clawToggle = !clawToggle;
        }
        clawActivate(clawToggle, dSolenoidClaw);

        // wheels claw vvv
        // Digital Tri-state Input (two boolean/button, or neither)
        double clawIntakeInput = (claw_intake ? 1 : 0) + (claw_expel ? -1 : 0);
        intake(clawIntakeInput, claw_Wheels);


        // Special Button Groupings vvv

        // Single Digital Input (one boolean/button)
        if(clawIntake_and_Extend){
            armExtend(1);
            intake(1, claw_Wheels);
        }

        // Single Digital Input (one boolean/button)
        if (slowClawWheels) {
            intake(clawIntakeSpeed_In_SLOW, claw_Wheels);        
        }
    }

    ////// Remove \/   

    public void lockWheel() {
        // TODO future challenge:
        // calculate the angle for each individual wheel to turn to given the
        // motorLocation and set each wheel to their respective lock value

        turnWheelToAngle(45);
    }

    public void resetPIDs() {
        pid.reset();
    }

    public void setContinouousInput() {
        pid.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void straightenWheel() {
        turnWheelToAngle(0);
    }

    public void turnWheelToAngle(double targetAngle) {
        double currentAngle = Math.abs(this.steerMotor.getAbsolutePosition());
        targetAngle = Math.abs(targetAngle);

        double tolerance = 0.0; // TODO move to config
        if (Math.abs(targetAngle - currentAngle) > tolerance) {
            double turnPower = pid.calculate(currentAngle, targetAngle);
            steerMotor.GO(turnPower);
        }
    }
}
