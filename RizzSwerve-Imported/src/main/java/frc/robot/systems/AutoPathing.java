package frc.robot.systems;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import com.kauailabs.navx.frc.AHRS;

public class AutoPathing{
    Driver driver;

    public void autoPathing(SwerveDriveKinematics kinematics, AHRS gyro, SwerveModulePosition swerveModPos){
        SwerveDriveOdometry odometer = new SwerveDriveOdometry(
            kinematics,
            gyro.Rotation2d(),
            swerveModPos
        );
       
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            Constants.maxVelocity_MetersPerSeconds, 
            Constants.maxAccel_MetersPerSecondsSquared
        );
        
    }   

    public Pose2d getPose(){ // pose2d holds x,y and roation 2d
        return odometer.PoseMeters();
    }

    public void resetOdometry(Pose2d pose){
        odometer.reset(pose, gyro.Rotation2d());
    }

    public void trajectoryGenerator(){// FIX THIS; WIP
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0, new Rotation2d(0)), // initial point
            list.of(
                new Translationd2d(1,0), // some more points
                new Translationd2d(1,-1) //another
            ),
            new Pose2d(0,0, Rotation2d.fromDegrees(180)), //final
                                                          // tl;dr spins 180 by the time it travels between the points
            trajectoryConfig
        );
    }

    public void pidTrajectoryTrackers(){//WIP, found online.... ????
        PIDController xController = new PIDController(null,null,null); //fix
        PIDController yController = new PIDController(null,null,null); //fix
        ProfiledPIDController thetaController = new ProfiledPIDController(
            //p,i,d,constarints?
        );
        thetaController.enableContinousInput(Math.Pi,Math.Pi);
    }

    public void followTrajectory(){// final
        SwerveControllerCommand swerveControllerCoommand = new SwerveControllerCommand(null,null,null,null,null,null,null
        //trajectory,
        //swerveSubsystem::getPose,
        //Drive kinematics,
        //xController,
        //yController,
        //swerveSubsystem::setModulesStates,
        //driveSystem
        );
    }
}




//public void updateOdometry(){
//    odometer.update(null,null // replace nulls with the following:
//        //Rotation2d()
//        //*4 module states: front left, -right, and then back left, -right
//    );
//}