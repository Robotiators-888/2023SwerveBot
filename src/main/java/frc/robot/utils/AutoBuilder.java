package frc.robot.utils;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotManager;
import frc.robot.Subsystems.SUB_Drivetrain;
import frc.robot.Subsystems.SUB_Extension;
import frc.robot.Subsystems.SUB_Intake;
import frc.robot.Subsystems.SUB_Manuiplator;

/** This utility class is built for selecting made autos */
public class AutoBuilder {
  SUB_Drivetrain drivetrain;
  SUB_Extension extension;
  SUB_Intake intake;
  SUB_Manuiplator manuiplator;
  RobotManager manager = RobotManager.getInstance();
  private final SendableChooser<Command> chooser = new SendableChooser<>();

  // ====================================================================
  //                          Trajectories
  // ====================================================================

  PathPlannerTrajectory dummyPath = PathPlannerBase.getTrajectory("DummyPath", false);
  PathPlannerTrajectory dummyDonut = PathPlannerBase.getTrajectory("DummyPathDonut", false);
  PathPlannerTrajectory figure8 = PathPlannerBase.getTrajectory("FigureThis", false);
  PathPlannerTrajectory score2Path = PathPlannerBase.getTrajectory("OutNBack", false);
  PathPlannerTrajectory score1NoCable = PathPlannerBase.getTrajectory("DriveBackNoCable", false);
  PathPlannerTrajectory score1Cable = PathPlannerBase.getTrajectory("DriveBackCable", false);


  // ====================================================================
  //                          Routines
  // ====================================================================

  private Command dummyPathOne() {
    return PathPlannerBase.followTrajectoryCommand(dummyPath, true);
  }

  private Command dummyPathDonut() {
    return PathPlannerBase.followTrajectoryCommand(dummyDonut, true);
  }

  private Command figureEight() {
    return PathPlannerBase.followTrajectoryCommand(figure8, true);
  }

  private Command score1BackNoCable(){
    return new SequentialCommandGroup(
      ScoreOne(),
      new WaitCommand(.5),
      PathPlannerBase.generateAuto(score1NoCable)
    );
  }

  private Command score1BackCable(){
    return new SequentialCommandGroup(
      ScoreOne(),
      new WaitCommand(.5),
      PathPlannerBase.generateAuto(score1Cable)
    );
  }

  private Command ScoreOne(){
    return new SequentialCommandGroup(
        new InstantCommand(()->manuiplator.setTargetPosition(manager.getScoringHeight(), manuiplator)),
        new WaitCommand(.5),
        extension.driveUntil(60, false),
        new RunCommand(()-> intake.runIntake(.6), intake).withTimeout(0.5),
        new InstantCommand(()->intake.runIntake(0)),
        returnManipulator()
    );
  }

  private Command nonCableSide2Pc(){
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("pickUp", new RunCommand(()->intake.runIntake(.60), intake));
    eventMap.put("stopPickup", new RunCommand(()->intake.runIntake(0.0)));
    return new SequentialCommandGroup(
        ScoreOne(),
        PathPlannerBase.generateAuto(eventMap, score2Path),
        ScoreOne()

    );
  }

  public AutoBuilder(SUB_Drivetrain drivetrain, SUB_Extension extension, SUB_Intake intake, SUB_Manuiplator manuiplator) {
    this.drivetrain = drivetrain;
    this.extension = extension;
    this.intake = intake;
    this.manuiplator = manuiplator;

    chooser.addOption("Dummy 1", dummyPathOne());
    chooser.addOption("Dummy Donut", dummyPathDonut());
    chooser.addOption("8's HEHEHEHE", figureEight());
    chooser.addOption("Score Two NonCable", nonCableSide2Pc());
    chooser.addOption("Score One NonCable", score1BackNoCable());
    chooser.addOption("Score One Cable", score1BackCable());


    chooser.setDefaultOption("Score One", ScoreOne());
    SmartDashboard.putData("Auto Selector", chooser);
  }

  /**
   * @return Returns chosen auto on Smartdashboard
   */
  public Command getSelectedAuto() {
    return chooser.getSelected();
  }

  // ====================================================================
  //                          Helpers
  // ====================================================================

  private Command returnManipulator(){
    return new SequentialCommandGroup(
        new InstantCommand(()->manuiplator.setTargetPosition(Constants.Manuiplator.kGroundPosition, manuiplator)),
        extension.driveUntil(1, true)

    );
  }

}
