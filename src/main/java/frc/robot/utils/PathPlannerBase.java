package frc.robot.utils;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Drivetrain;
import frc.robot.Subsystems.SUB_Drivetrain;

public class PathPlannerBase {
    

    final static SUB_Drivetrain drivetrain = RobotContainer.drivetrain;
    final static PathConstraints constraints = new PathConstraints(2, 2);


    public static PathPlannerTrajectory getTrajectory(String plannerFile, boolean reversed) {
        PathPlannerTrajectory trajectoryPath;
            trajectoryPath = PathPlanner.loadPath(plannerFile, constraints, reversed); //Filesystem.getDeployDirectory().toPath().resolve(plannerFile);
        

        return trajectoryPath;
    }
    
    public static Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
             new InstantCommand(() -> {
               // Reset odometry for the first path you run during auto
               if(isFirstPath){
                drivetrain.resetOdometry(traj.getInitialHolonomicPose());
               }
             }),
             new PPSwerveControllerCommand(
                 traj, 
                 drivetrain::getPose, // Pose supplier
                 Drivetrain.kDriveKinematics, // SwerveDriveKinematics
                 new PIDController(5, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 new PIDController(5, 0, 0), // Y controller (usually the same values as X controller)
                 new PIDController(3, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 drivetrain::setModuleStates, // Module states consumer
                 true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                 drivetrain // Requires this drive subsystem
             )
         );
     }

     public static Command generateAuto(PathPlannerTrajectory traj){
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(drivetrain::getPose, 
            drivetrain::resetOdometry,
            Constants.Drivetrain.kDriveKinematics,
            new PIDConstants(0.0, 0.0, 0.0), 
            new PIDConstants(0.0, 0.0, 0.0), 
            drivetrain::setModuleStates,  
            new HashMap<String, Command>(), 
            true);
         return autoBuilder.fullAuto(traj);
     }

     public static Command generateAuto(HashMap<String, Command> eventMap, PathPlannerTrajectory traj){
        
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(drivetrain::getPose, 
            drivetrain::resetOdometry,
            Constants.Drivetrain.kDriveKinematics,
            new PIDConstants(0.0, 0.0, 0.0), 
            new PIDConstants(0.5, 0.0, 0.0), 
            drivetrain::setModuleStates,  
            eventMap, 
            true);
         return autoBuilder.fullAuto(traj);

    }

    public static Command generateAuto(HashMap<String, Command> eventMap, PathPlannerTrajectory traj, StopEvent stopEvent){
        
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(drivetrain::getPose, 
            drivetrain::resetOdometry,
            Constants.Drivetrain.kDriveKinematics,
            new PIDConstants(5.0, 0.0, 0.0), 
            new PIDConstants(0.5, 0.0, 0.0), 
            drivetrain::setModuleStates,  
            eventMap, 
            true);
            autoBuilder.stopEventGroup(stopEvent);
         return autoBuilder.fullAuto(traj);

    }

}
