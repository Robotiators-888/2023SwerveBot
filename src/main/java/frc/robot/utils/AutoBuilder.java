package frc.robot.utils;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.SUB_Drivetrain;

public class AutoBuilder {
    SUB_Drivetrain drivetrain;
    private final SendableChooser<Command> chooser = new SendableChooser<>();

    // ====================================================================
    //                          Trajectories
    // ====================================================================

    PathPlannerTrajectory dummyPath = PathPlannerBase.getTrajectory("DummyPath", false);
    PathPlannerTrajectory dummyDonut = PathPlannerBase.getTrajectory("DummyPathDonut", false);
    PathPlannerTrajectory figure8 = PathPlannerBase.getTrajectory("FigureThis", false);



    // ====================================================================
    //                          Routines
    // ====================================================================

    private Command dummyPathOne(){
        return PathPlannerBase.followTrajectoryCommand(dummyPath, true);
    }

    private Command dummyPathDonut(){
        return PathPlannerBase.followTrajectoryCommand(dummyDonut, true);
    }

    private Command figureEight(){
        return PathPlannerBase.followTrajectoryCommand(figure8, true);
    }



    public AutoBuilder(SUB_Drivetrain drivetrain){
        this.drivetrain = drivetrain;

        chooser.addOption("Dummy 1", dummyPathOne());
        chooser.addOption("Dummy Donut", dummyPathDonut());
        chooser.addOption("8's HEHEHEHE", figureEight());

        SmartDashboard.putData("Auto Selector", chooser);


    }

    public Command getSelectedAuto(){
        return chooser.getSelected();
    }
}
