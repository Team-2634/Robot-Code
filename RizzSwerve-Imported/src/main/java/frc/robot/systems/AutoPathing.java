//git merge develop
//git checkout <your-feature-branch>

//READ ME: I am not sure if the module state, gyro and such which require to be updated regularly will be... keep that in mind!! 
//FURTHERMORE: reset and update method exists if needed, however it dose not re-get the gyro and modules, just their values... I think?
//ONE MORE THINGS: pid values = 0 at the time of writing this :P
package frc.robot.systems;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import com.kauailabs.navx.frc.AHRS;

import java.util.Arrays;

public class AutoPathing{
    int PID_DEAFULT = 0;//switch to conts
    add driver thing back, my bad I delete it :(
    private SwerveDriveKinematics kinematics;
    private AHRS gyro;
    private SwerveModulePosition[] swerveModPos;
    
    public AutoPathing(SwerveDriveKinematics kinematics, AHRS gyro, SwerveModulePosition[] swerveModPos) {
        this.kinematics = kinematics;
        this.gyro = gyro;
        this.swerveModPos = swerveModPos;
    }
    
    SwerveDriveOdometry odometer = new SwerveDriveOdometry(//dead reckoning: Where am I, Then where do I need to be
        kinematics, 
        gyro.getRotation2d(), 
        swerveModPos
    );

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(//motion planning: Where to next, While asking: where on the path am I?
        Constants.maxVelocity_MetersPerSeconds, 
        Constants.maxAccel_MetersPerSecondsSquared
    );

    public Pose2d getPose(){ // pose2d holds x,y and roation 2d
        return odometer.getPoseMeters();
    }

    public void updateOdometry(){
        odometer.update(gyro.getRotation2d(), swerveModPos);
    }

    public void resetOdometry(Pose2d pose){ // this resest current location too, see pose2d
        odometer.resetPosition(gyro.getRotation2d(), swerveModPos, pose);
    }

    public void trajectoryGenerator(){// change this in usch a way we can create auto code by calling this method.
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0, new Rotation2d(0)), // initial point
            Arrays.asList(
                new Translation2d(1,0), // some more points
                new Translation2d(1,-1) //another
            ),
            new Pose2d(0,0, Rotation2d.fromDegrees(180)), //final
                                                                        // tl;dr spins 180 by the time it travels between the points
            trajectoryConfig
        );
    }

    public void pidTrajectoryTrackers(){//combine the functionality of PID control and motion profiling. aka generate setpoints using a motion profile and then use a PID controller to follow those setpoints.
        PIDController xController = new PIDController(PID_DEAFULT, PID_DEAFULT, PID_DEAFULT);
        PIDController yController = new PIDController(PID_DEAFULT, PID_DEAFULT, PID_DEAFULT);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            PID_DEAFULT,
            PID_DEAFULT,
            PID_DEAFULT,
            new TrapezoidProfile.Constraints(Constants.maxVelocity_MetersPerSeconds, Constants.maxAccel_MetersPerSecondsSquared)
        );
        thetaController.enableContinuousInput(Math.PI,Math.PI);
        return new PIDCONTROLLER[]{xController,yController}; asjdfhgasjkfgasd // fix
    }

    public void followTrajectory(){// final
        SwerveControllerCommand swerveControllerCoommand = new SwerveControllerCommand(
        trajectory,
        this::getPose,
        kinematics,
        xController,
        yController,
        //swerveSubsystem::setModulesStates,
        driver
        );
    }
}
/* Here's a simplified view of how it works:

    A Trajectory is generated using a trajectory generator, which calculates a path for the robot to follow.
    The Trajectory is converted into a series of SwerveModuleState objects using the SwerveDriveKinematics class. This involves calculating the desired speed and angle for each module based on the trajectory.
    The SwerveControllerCommand takes these SwerveModuleState objects and commands the individual swerve modules to achieve those states.

another: 
CommandScheduler scheduler = CommandScheduler.getInstance();
scheduler.schedule(swerveControllerCommand);
or
private SwerveControllerCommand swerveControllerCommand;

@Override
public void autonomousInit() {
    Trajectory trajectory = trajectoryGenerator();
    PIDController[] controllers = pidTrajectoryTrackers();
    swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        this::getPose,
        kinematics,
        controllers[0],
        controllers[1],
        swerveModPos::setState,
        driver
    );
    swerveControllerCommand.initialize();
}

@Override
public void autonomousPeriodic() {
    swerveControllerCommand.execute();
}

*/