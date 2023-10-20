package frc.robot.Commands;


import frc.robot.Subsystems.SUB_Drivetrain;
import frc.robot.Subsystems.SUB_Limelight;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CMD_LimeAlign extends CommandBase {
  SUB_Limelight limelight;
  SUB_Drivetrain drivetrain;

  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3,2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3,2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

  private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);
  
  /**
   * Drive to goal using limelight
   * @param limeIn
   * @param driveIn
   */
  public CMD_LimeAlign(SUB_Limelight limeIn, SUB_Drivetrain driveIn) {
    this.limelight = limeIn;
    this.drivetrain = driveIn;
    addRequirements(limeIn, driveIn);
  }

  @Override
  public void initialize() {
    limelight.setLed(3);
  }

  @Override
  public void execute() {
    var targetTransform = limelight.getTargetTransform();
    var robotPose = drivetrain.getPose();


    if (targetTransform != null){
        var goalPose = robotPose.transformBy(targetTransform);
        
        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        omegaController.setGoal(goalPose.getRotation().getRadians());

        var xSpeed = xController.calculate(robotPose.getX());
        if (xController.atGoal()){
          xSpeed = 0;
        }

        var ySpeed = yController.calculate(robotPose.getY());
        if (yController.atGoal()){
          ySpeed = 0;
        }

        var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());
        if (omegaController.atGoal()){
          omegaSpeed = 0;
        }

        drivetrain.drive(xSpeed, ySpeed, omegaSpeed, true, true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // drivetrain.setMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}