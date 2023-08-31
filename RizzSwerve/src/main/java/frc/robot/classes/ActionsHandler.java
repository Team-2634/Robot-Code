package frc.robot.classes;

import com.kauailabs.navx.frc.AHRS;

public class ActionsHandler {

    public static void ActionRunner(SwerveDriveTrain driveTrain, AHRS navx, Motor shouldeMotor, Motor armExtension) {
        try {
            //Thread t1 = new Thread(new actions(driveTrain, shouldeMotor, armExtension, "drive", 10));
            //Thread t2 = new Thread(new actions(driveTrain, shouldeMotor, armExtension, "moveArm", 15));
            //Thread t3 = new Thread(new actions(driveTrain, shouldeMotor, armExtension, "placeThingy", 1));
            //Thread t4 = new Thread(new actions(driveTrain, shouldeMotor, armExtension, "DriveBackwardsThenRotate", 10));

            Thread t1 = new Thread(new actions(driveTrain, navx, shouldeMotor, armExtension, "driveRotate", Math.PI - 0.01));
            Thread t2 = new Thread(new actions(driveTrain, navx, shouldeMotor, armExtension, "driveDistance", 0.21));
            Thread t3 = new Thread(new actions(driveTrain, navx, shouldeMotor, armExtension, "placeThingy", 1));
            Thread t4 = new Thread(new actions(driveTrain, navx, shouldeMotor, armExtension, "DriveBackwardsThenRotate", 10));


            t1.start(); // starts thread
            t1.join();
            driveTrain.setDriveEncoderPosition(0);;

            t2.start();
            t2.join();


            

            t1.join(); // wait here until thread is finished
            t2.join();

            t3.start();
            t3.join();

            t4.start();
            t4.join();

            System.out.println("Fin!");

        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}