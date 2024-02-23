package frc.robot.systems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;

public class Shooter {
    private final CANSparkMax intake = new CANSparkMax(9, MotorType.kBrushless);

    private final CANSparkMax shooterMotorLeft = new CANSparkMax(0, MotorType.kBrushless);
    private final CANSparkMax shooterMotorRight = new CANSparkMax(0, MotorType.kBrushless);
    
    // private final TalonFX ShooterMotorLeft = new TalonFX(0);
    // private final TalonFX ShooterMotorRight = new TalonFX(0);
    
    // private final TalonFX ArmMotor_L = new TalonFX(0);
    // private final TalonFX ArmMotor_R = new TalonFX(0);

    // private final TalonFX PickUpMotor = new TalonFX(0);
    
    // Timer LaunchTimer;
    public void collectNote(boolean condition){
        if (condition) {
            intake.set(0.5);    
        }
        else {
            intake.set(0);
        }
    }
   
    // public void RotateArm(double value){
    //     ArmMotor_L.set(value);
    //     ArmMotor_R.set(value);
    // }

    public void shootNote(boolean condition //double seconds, double speed
    ) {
        if (condition) {
            shooterMotorLeft.set(0.5);
            shooterMotorRight.set(0.5);
        }
        else {
            shooterMotorLeft.set(0);
            shooterMotorRight.set(0);
        }
        
        // LaunchTimer.reset();
        // LaunchTimer.start();
        // if (LaunchTimer.get() > seconds){
        //     ShooterMotor_L.set(speed);
        //     ShooterMotor_R.set(speed);
        // }
        
        // ShooterMotor_L.set(0.0);
        // ShooterMotor_R.set(0.0);

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