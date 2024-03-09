package frc.robot.systems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class Shooter {
    // private final CANSparkMax intake = new CANSparkMax(9, MotorType.kBrushless);

    // private final CANSparkMax shooterMotorLeft = new CANSparkMax(0, MotorType.kBrushless);
    // private final CANSparkMax shooterMotorRight = new CANSparkMax(0, MotorType.kBrushless);

    // private final CANSparkMax armMotorLeft = new CANSparkMax(0, MotorType.kBrushless);
    // private final CANSparkMax armMotorRight = new CANSparkMax(0, MotorType.kBrushless);
    
    private final TalonFX shooterMotorLeft = new TalonFX(Constants.leftShootID, Constants.canivore); 
    private final TalonFX shooterMotorRight = new TalonFX(Constants.rightShootID, Constants.canivore);
    
    public final TalonFX armMotorLeft = new TalonFX(Constants.leftArmID, Constants.canivore);
    public final TalonFX armMotorRight = new TalonFX(Constants.rightArmID, Constants.canivore);

    private final TalonFX intake = new TalonFX(Constants.intakeID, Constants.canivore);

    private final ArmFeedforward armFF = new ArmFeedforward(0, 0, 0);
    private final PIDController armPID = new PIDController(Constants.kpArm, Constants.kiArm, Constants.kdArm);
    
    public void initialize() {
        armMotorLeft.setNeutralMode(NeutralModeValue.Brake);
        armMotorRight.setNeutralMode(NeutralModeValue.Brake);

        armMotorLeft.setInverted(true);
        armMotorRight.setInverted(false);
    }

    // Timer LaunchTimer;
    public void collectNote(double speed){
        intake.set(speed);
    }
   
    // public void RotateArm(double value){
    //     ArmMotor_L.set(value);
    //     ArmMotor_R.set(value);
    // }

    public void shootNote(double speed) {
        shooterMotorLeft.set(-speed);
        shooterMotorRight.set(-speed);
    }

    // public void shootNoteFull() {
    //     shootNote(1);
    //     if (shooterMotorLeft.getRotorVelocity().getValue()>100 && shooterMotorRight.getRotorVelocity().getValue()>100) {
            
    //         collectNote(0.33);
    //     }
    // }

    public void moveArm(double speed) {
        armMotorLeft.set(speed);
        armMotorRight.set(speed);
    }

    public double getArmRadians() {
        return armMotorLeft.getPosition().getValue() * Constants.armGearRatio * 2 * Math.PI;
    }

    public void moveArmPID(double position) {
        double power = armPID.calculate(getArmRadians(), position); // + armFF.calculate(getArmRadians() - Constants.armOffset, position - Constants.armOffset);
        armMotorLeft.set(power);
        armMotorRight.set(power);
    }

    public boolean isHardStoppedLow() {
        if (2 < armMotorLeft.getPosition().getValue()) {
            return false;
        } else return true;
    }

    public boolean isHardStoppedHigh() {
        if (armMotorLeft.getPosition().getValue() < Constants.maxArmRotation) {
            return false;
        } else return true;
    }
    // ColorSensorV3 noteSensor = new ColorSensorV3();
    // public boolean colourSensorDetected() {

    // }
    
}
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⣀⣤⣤⣤⣴⢶⣴⡶⣶⣶⣴⣤⣤⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⣤⣤⣶⣾⢿⣿⣻⣟⣯⣿⣽⣯⣿⣾⣿⣿⣿⣿⣿⣿⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣠⣤⣶⣿⢿⣻⣯⣟⣷⢯⣿⣞⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣴⣾⡿⣟⡿⣽⡾⣟⡿⣾⣽⢾⣿⣷⣿⣿⣻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⣀⣴⣿⣟⣯⣷⢿⣻⣽⢿⣽⣻⣽⣷⣿⣿⣟⣿⣿⣳⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⣠⣾⢿⣯⢷⣻⣽⡾⣟⣯⣟⡿⣞⣯⣷⢿⣳⣟⣾⢿⣿⢯⣟⣿⣿⣿⡿⣿⣿⣿⣿⣿⣿⡿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⣠⣾⢿⣽⣻⡾⣟⣯⣷⢿⣯⣟⣾⣟⡿⣽⢯⣟⡿⣞⣯⡿⣿⡿⣯⣟⣿⣿⣿⣿⣿⣿⣿⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⣠⣿⣟⣿⡼⣟⣿⢿⣻⣼⡿⣼⣻⣧⣟⣿⢿⣟⣿⣻⣿⣻⢿⣿⣿⢿⣼⣿⣿⣿⣼⣿⣿⣿⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⣴⣿⣻⡾⣽⣻⣟⣾⣟⡿⣾⣽⣿⣿⣳⡿⣾⣻⣽⣾⣻⢾⣽⣟⣾⣿⢿⣞⣿⣿⣿⣟⣿⣿⣿⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⢰⣿⣯⣷⢿⣻⢷⣯⣷⣻⣽⣷⣿⣿⣿⣿⣿⣳⣿⣳⣯⣟⣯⣷⣟⡷⣿⣿⣻⢾⣿⣿⣿⣻⣿⣿⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠻⢿⣷⣿⣯⣿⣿⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣞⡷⣟⣾⣟⣾⣽⣻⣽⢿⣟⣯⣿⣿⣿⣟⣿⣿⣿⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⡄⠀⠀⠀⠀⠀
// ⠀⠀⠈⠙⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢯⣿⣻⢷⣻⣾⣳⡿⣽⣻⣿⣟⡾⣿⣿⣿⣿⣿⣿⣿⣿⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⢠⣾⣻⠃⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠙⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣯⢷⣟⡿⣽⡾⣯⣟⣿⣳⣿⣿⣻⣽⣿⣿⣾⣿⣿⣿⣿⣿⣀⣠⣤⣴⣶⣴⣶⣶⣶⣶⣾⠟⠋⠁⣠⣞⡋⠁⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠙⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣯⡿⣯⣟⣷⣟⡷⣟⣾⢿⣿⣳⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡟⠁⠀⠀⣼⣿⣿⣿⣷⣦⣀⠠⣤⡦⠤
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠹⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣟⣷⣟⡷⣯⣿⣻⣽⣻⣿⣿⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠟⣿⣿⣿⣀⣶⣶⣾⣿⣿⣿⣿⠟⢉⣤⡞⠁⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣞⣿⣽⢾⣯⢷⣟⣾⣿⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠟⣡⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠃⣰⣿⣿⣷⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢾⡽⣟⣾⣿⡿⣟⣯⣿⡿⣯⣿⣿⣿⣿⡿⠛⠉⢠⣾⣿⣿⣿⢿⣻⣯⣿⣞⣿⣿⡿⠃⣽⣿⣿⣿⣿⡇⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠹⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢿⣳⣿⣻⣽⣷⣿⣿⣿⣿⡿⠋⠀⠀⠀⣿⣿⢿⣽⡞⢁⣿⣟⣷⣻⣿⠟⠁⣸⣿⣿⣿⣿⣿⡇⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢿⣽⣻⣽⣾⣿⣻⣿⣿⣿⣿⣿⠁⠀⠀⠀⠀⣿⣽⣯⠏⠀⢸⣿⣻⡾⠋⠁⠀⢠⣿⣿⣿⣿⣿⣿⠃⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⢻⣿⣿⣿⣿⣿⡿⣟⣯⣿⣻⣞⣯⣷⡿⣽⣿⣿⣿⣿⣿⣏⠀⠀⠀⠀⠀⠙⢷⣿⠅⠀⠀⡈⠐⠀⠀⠀⢠⣾⣿⣿⣿⣿⣿⡟⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⣿⣿⣿⣿⣿⣿⢯⣟⣾⢷⣻⣽⣾⣻⣿⡿⣻⣿⢿⡽⡟⠀⠀⠀⠀⠀⠀⢀⣤⣶⣶⣿⣷⣤⣤⣤⣾⣿⣿⣿⣿⣿⣿⡿⠁⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣤⣴⣾⣿⣿⣿⣿⣿⣿⣿⣽⢾⣟⣯⣷⢯⣿⡟⢡⣿⣯⣿⣻⠏⠀⠀⠀⠀⠀⣸⣿⣿⣻⣽⡾⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠁⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⢀⣤⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣾⣻⣽⣾⣯⣿⣿⠁⠀⢻⣷⣿⠋⠀⠀⠀⠀⠀⢠⣿⣟⣾⣯⣷⢿⣿⣿⣿⣿⣿⣿⣿⡿⠻⠋⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⣠⣾⣿⣿⢿⣟⡿⣯⣟⣯⣿⣿⣿⣿⣿⣿⣿⣷⣿⣽⣿⣿⣿⡏⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⢛⣿⣟⣾⣿⣿⣿⣿⣿⣿⡿⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⢠⣾⣿⣟⡿⣾⢯⣿⡽⣷⣻⣽⣞⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠏⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣴⣿⢿⣽⣾⣿⣿⣿⣿⠟⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⣿⣿⣽⣾⢻⣽⣿⡞⣿⣽⢻⣾⣽⢻⣽⣿⣿⣿⣿⣿⣿⣿⣯⣿⣤⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⣤⣶⣿⣿⣯⣿⣿⣿⣿⡟⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⢿⣟⣷⣯⢿⣻⣾⡽⣿⡽⣯⣷⢿⣻⣟⣾⣽⣻⣟⣯⣿⣿⢾⡿⡿⠁⠀⠀⠀⠀⠀⠀⣠⣶⣿⢿⣻⣿⡷⣯⣿⠟⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠈⠻⣷⣯⢿⣻⡾⣽⡷⣟⣿⢾⣻⣽⣾⣻⣾⢷⣿⣿⣿⡽⣯⡟⠁⠀⠀⠀⠀⠀⠀⠈⠛⠟⠛⢛⣿⡿⣽⡿⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠈⠻⣻⡷⣟⣯⣿⣻⢾⣿⢿⣻⣿⣿⣿⣿⣿⣿⣳⡿⣿⠀⢀⣠⣤⣤⣄⠀⠀⠀⣀⣤⣴⣿⣿⣽⠟⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠈⠛⠿⠾⣿⣿⣿⣿⣟⠛⠛⠛⢿⣿⣿⣿⡽⣟⣶⣿⢿⣻⣯⢿⣿⢾⡿⣿⣻⢯⣷⠟⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠹⣿⣷⣤⡐⠀⣻⣿⣿⣻⣟⣯⣟⣿⣳⢿⣻⡾⣿⡽⣷⣿⠟⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠻⢿⣿⣿⣿⣿⡟⠉⠉⠉⠙⠚⠻⢿⣽⣿⣷⡿⠋⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠛⠿⠿⠇⠀⠀⠀⠀⠀⠀⠀⠈⠉⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀

//                                 TAKE YOUR HEART