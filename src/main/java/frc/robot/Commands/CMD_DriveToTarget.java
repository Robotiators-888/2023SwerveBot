package frc.robot.Commands;


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

  private final PIDController xController = new PIDController(0, 0, 0); // 3, 0, 0
  private final PIDController yController = new PIDController(0, 0, 0); // 3, 0, 0
  private final PIDController omegaController = new PIDController(0, 0, 0); // 2, 0, 0
  
  /**
   * Drive to goal using limelight
   * @param limelight The limelight subsystem
   * @param drivetrain The drivetrain subsystem
   */
  public CMD_DriveToTarget(SUB_Limelight limelight, SUB_Drivetrain drivetrain) {
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    addRequirements(limelight, drivetrain);
  }

  @Override
  public void initialize() {
    limelight.setPipline(0);
  }

  @Override
  public void execute() {
    var targetTransform = limelight.getTargetTransform();
    var robotPose = drivetrain.getPose();

    if (targetTransform != null){
        var goalPose = robotPose.transformBy(targetTransform);
        SmartDashboard.putBoolean("Has Target", true);
        SmartDashboard.putNumber("X Error", goalPose.getX() - robotPose.getX());
        SmartDashboard.putNumber("Y Error", goalPose.getY() - robotPose.getY());
        SmartDashboard.putNumber("Z Error", goalPose.getRotation().getRadians() - robotPose.getRotation().getRadians());

        var xSpeed = xController.calculate(robotPose.getX(), goalPose.getX());
        var ySpeed = yController.calculate(robotPose.getY(), goalPose.getY());
        var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians(), goalPose.getRotation().getRadians());

        drivetrain.drive(xSpeed, ySpeed, omegaSpeed, true, true);
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
    // return (xController.atSetpoint() && yController.atSetpoint() && omegaController.atSetpoint())
    return false;
  }
}