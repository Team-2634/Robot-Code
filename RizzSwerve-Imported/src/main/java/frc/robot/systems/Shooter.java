package frc.robot.systems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
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
    
    private final TalonFX armMotorLeft = new TalonFX(Constants.leftArmID, Constants.canivore);
    private final TalonFX armMotorRight = new TalonFX(Constants.rightArmID, Constants.canivore);

    private final TalonFX intake = new TalonFX(Constants.intakeID, Constants.canivore);

    private final ArmFeedforward armFF = new ArmFeedforward(0, 0, 0);
    private final PIDController armPID = new PIDController(0, 0, 0);
    
    // Timer LaunchTimer;
    public void collectNote(double speed){
        intake.set(speed);
    }
   
    // public void RotateArm(double value){
    //     ArmMotor_L.set(value);
    //     ArmMotor_R.set(value);
    // }

    public void shootNote(double speed //double seconds, double speed
    ) {
        shooterMotorLeft.set(speed);
        shooterMotorRight.set(speed);
        
        // LaunchTimer.reset();
        // LaunchTimer.start();
        // if (LaunchTimer.get() > seconds){
        //     ShooterMotor_L.set(speed);
        //     ShooterMotor_R.set(speed);
        // }
        
        // ShooterMotor_L.set(0.0);
        // ShooterMotor_R.set(0.0);

    }

    public void moveArm(double speed) {
        armMotorLeft.set(speed);
        armMotorRight.set(speed);
    }

    public double getArmRadians() {
        return armMotorLeft.getPosition().getValueAsDouble() * Constants.armGearRatio / 2 / Math.PI;
    }

    public void moveArmPID(double position) {
        double power = armFF.calculate(getArmRadians() - Constants.armOffset, position) + armPID.calculate(getArmRadians(), position);
        armMotorLeft.setVoltage(power);
        armMotorRight.setVoltage(power);
    }

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