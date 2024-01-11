package frc.robot.Commands;


import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.SUB_Drivetrain;
import frc.robot.Subsystems.SUB_Limelight;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CMD_DriveToTarget extends CommandBase {
  SUB_Limelight limelight;
  SUB_Drivetrain drivetrain;

  // private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3,2);
  // private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3,2);
  // private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

  private final PIDController xController = new PIDController(0.3, 0, 2); // 3, 0, 0
  private final PIDController yController = new PIDController(3/5, 0, 0); // 3, 0, 0
  private final PIDController omegaController = new PIDController( 0.25, 0, 0); // 2, 0, 0
  
  /**
   * Drive to goal using limelight
   * @param limelight The limelight subsystem
   * @param drivetrain The drivetrain subsystem
   */
  public CMD_DriveToTarget(SUB_Limelight limelight, SUB_Drivetrain drivetrain) {
    this.limelight = limelight;
    this.drivetrain = drivetrain;

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
    var targetTransform = limelight.getTargetTransform();

    if (targetTransform != null){
        SmartDashboard.putNumber("ANGLE", targetTransform.getRotation().getAngle());
        SmartDashboard.putNumber("GOAL POSE Z", targetTransform.getZ());
        SmartDashboard.putNumber("GOAL POSE X", targetTransform.getX());

        var xSpeed = xController.calculate(0, targetTransform.getZ()-1);
        var ySpeed = yController.calculate(0, targetTransform.getX());
       var omegaSpeed = omegaController.calculate(0, targetTransform.getRotation().getY());

        // Robot Pose: + X is forward, + Y is to the left, + Theta is counterclockwise
        drivetrain.drive(xSpeed, -ySpeed, -omegaSpeed, false, true);
    } else {
      SmartDashboard.putBoolean("Has Target", false);
    }
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