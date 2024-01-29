//git merge develop
//git checkout <your-feature-branch>

//READ ME: I am not sure if the module state, gyro and such which require to be updated regularly... keep that in mind!! 
//FURTHERMORE: reset and update method exists if needed, however it dose not re-get the gyro and modules, just their values... I think?
//ONE MORE THINGS: pid values = 0 at the time of writing this :P
package frc.robot.systems;

import frc.robot.Constants;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import com.kauailabs.navx.frc.AHRS;

import java.util.Arrays;

public class AutoPathing{
    int PID_DEAFULT = 0;//switch to conts
    Driver driver = new Driver();
    private SwerveDriveKinematics kinematics;
    private AHRS gyro;
    private SwerveModulePosition[] swerveModPos;
    private SwerveModuleState[] moduleStatesArray;
    Trajectory trajectory;
    PIDController xController, yController;
    ProfiledPIDController thetaController;
    
    public AutoPathing(SwerveDriveKinematics kinematics, AHRS gyro, SwerveModulePosition[] swerveModPos, SwerveModuleState[] moduleStatesAry) {
        this.kinematics = kinematics;
        this.gyro = gyro;
        this.swerveModPos = swerveModPos;
        this.moduleStatesArray = moduleStatesAry;
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
        trajectory = TrajectoryGenerator.generateTrajectory(
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
        xController = new PIDController(PID_DEAFULT, PID_DEAFULT, PID_DEAFULT);
        yController = new PIDController(PID_DEAFULT, PID_DEAFULT, PID_DEAFULT);
        thetaController = new ProfiledPIDController(
            PID_DEAFULT,
            PID_DEAFULT,
            PID_DEAFULT,
            new TrapezoidProfile.Constraints(Constants.maxVelocity_MetersPerSeconds, Constants.maxAccel_MetersPerSecondsSquared)
        );
        thetaController.enableContinuousInput(Math.PI,Math.PI);
        
        //Feedforward control is a technique used to compensate for static friction and other forces that cause the motor to require more power to overcome inertia
        //SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 3, 1);
    }

    public void followTrajectory(){
        HolonomicDriveController holoController = new HolonomicDriveController(xController, yController, thetaController);
        SwerveControllerCommand swerveControllerCoommand = new SwerveControllerCommand(
            trajectory, 
            this::getPose, 
            kinematics, 
            holoController, 
            null, // need module states!!
             null);
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

an another thing: 
// Create a SwerveDriveKinematics object
List<Translation2d> moduleLocations = Arrays.asList(
new Translation2d(Constants.kTrackWidth / 2.0, Constants.kWheelBase / 2.0),
new Translation2d(Constants.kTrackWidth / 2.0, -Constants.kWheelBase / 2.0),
new Translation2d(-Constants.kTrackWidth / 2.0, -Constants.kWheelBase / 2.0),
new Translation2d(-Constants.kTrackWidth / 2.0, Constants.kWheelBase / 2.0)
);

SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(moduleLocations);

// Create a SwerveDriveOdometry object
SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation());

// Generate a trajectory
TrajectoryConfig config = new TrajectoryConfig(5, 5)
.setReversed(false)
.addConstraint(new MaxVelocityConstraint(Units.feetToMeters(12)))
.addConstraint(new CentripetalAccelerationConstraint(Math.pow(Units.feetToMeters(12), 2) / Math.pow(Constants.kMaxRadius, 2)));

Trajectory trajectory = TrajectoryGenerator.generateTrajectory(config, List.of(), new Pose2d());

// Follow the trajectory
SwerveControllerCommand command = new SwerveControllerCommand(
trajectory,
() -> m_odometry.getPoseMeters(),
m_kinematics,
(state) -> {
// Set the desired output of each swerve module here
},
m_drive
);

command.schedule();

// Update the odometry object in your main loop
m_odometry.update(getGyroscopeRotation(), getModuleStates());


phind:
The `SwerveControllerCommand` class in the WPILib library has three constructors, each requiring different parameters. Here are the parameters for each constructor:

1. **Constructor 1:**
   - `Trajectory trajectory`: The trajectory to follow.
   - `Supplier<Pose2d> pose`: A function that supplies the robot pose.
   - `SwerveDriveKinematics kinematics`: The kinematics for the robot drivetrain.
   - `PIDController xController`: The Trajectory Tracker PID controller for the robot's x position.
   - `PIDController yController`: The Trajectory Tracker PID controller for the robot's y position.
   - `ProfiledPIDController thetaController`: The Trajectory Tracker PID controller for angle for the robot.
   - `Supplier<Rotation2d> desiredRotation`: The angle that the drivetrain should be facing. This is sampled at each time step.
   - `Consumer<SwerveModuleState[]> outputModuleStates`: The raw output module states from the position controllers.
   - `Subsystem... requirements`: The subsystems to require.

2. **Constructor 2:**
   - `Trajectory trajectory`: The trajectory to follow.
   - `Supplier<Pose2d> pose`: A function that supplies the robot pose.
   - `SwerveDriveKinematics kinematics`: The kinematics for the robot drivetrain.
   - `PIDController xController`: The Trajectory Tracker PID controller for the robot's x position.
   - `PIDController yController`: The Trajectory Tracker PID controller for the robot's y position.
   - `ProfiledPIDController thetaController`: The Trajectory Tracker PID controller for angle for the robot.
   - `Consumer<SwerveModuleState[]> outputModuleStates`: The raw output module states from the position controllers.
   - `Subsystem... requirements`: The subsystems to require.

3. **Constructor 3:**
   - `Trajectory trajectory`: The trajectory to follow.
   - `Supplier<Pose2d> pose`: A function that supplies the robot pose.
   - `SwerveDriveKinematics kinematics`: The kinematics for the robot drivetrain.
   - `HolonomicDriveController controller`: The HolonomicDriveController for the drivetrain.
   - `Consumer<SwerveModuleState[]> outputModuleStates`: The raw output module states from the position controllers.
   - `Subsystem... requirements`: The subsystems to require.

Please note that the second and third constructors assume that the final rotation of the robot will be set to the rotation of the final pose in the trajectory. The robot will not follow the rotations from the poses at each timestep. If alternate rotation behavior is desired, the first constructor with a supplier for rotation should be used.

*/