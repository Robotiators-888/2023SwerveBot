package frc.robot.Commands;


import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.SUB_Drivetrain;
import frc.robot.Subsystems.SUB_Limelight;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CMD_AbsoluteDriveToTarget extends CommandBase {
  SUB_Limelight limelight;
  SUB_Drivetrain drivetrain;

  // private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3,2);
  // private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3,2);
  // private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

  private final PIDController xController = new PIDController(0.3, 0, 2); // 3, 0, 0
  private final PIDController yController = new PIDController(3/5, 0, 0); // 3, 0, 0
  private final PIDController omegaController = new PIDController( 0.25, 0, 0); // 2, 0, 0
  private final Pose2d goalPose;
  
  /**
   * Drive to goal using limelight
   * @param limelight The limelight subsystem
   * @param drivetrain The drivetrain subsystem
   */
  public CMD_AbsoluteDriveToTarget(SUB_Limelight limelight, SUB_Drivetrain drivetrain, Pose2d goalPose) {
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    this.goalPose = goalPose;

    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    omegaController.setTolerance(0.05);
    addRequirements(limelight, drivetrain);
  }

  @Override
  public void initialize() {
    limelight.setPipeline(0);
  }

  @Override
  public void execute() {
    var robotPose = drivetrain.getPose();

    var xSpeed = xController.calculate(robotPose.getX(), goalPose.getX());
    var ySpeed = yController.calculate(robotPose.getX(), goalPose.getY());
    // In this case, it has to be what orientation we want the robot to be in.
    var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians(), goalPose.getRotation().getRadians());

    // Robot Pose: + X is forward, + Y is to the left, + Theta is counterclockwise
    drivetrain.drive(xSpeed, -ySpeed, -omegaSpeed, false, true);
  }

  @Override
  public void end(boolean interrupted) {
    // drivetrain.setMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (xController.atSetpoint() && yController.atSetpoint() && omegaController.atSetpoint());
  }
}