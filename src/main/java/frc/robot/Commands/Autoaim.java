package frc.robot.commands;


public class AutoAimCommand extends CommandBase {
  private static SwerveDrive swerveDrive;
  private static XboxController remote;
  private static Shooter shooter;
  private static Hood hood;
  private static InnerIndex inner;
  private static OuterIndex outer;
  private static Limelight light;
  private double p = 0.004;
  private double i = 0;
  private double d = 0.00001;

  public AutoAimCommand() {
    light = RobotContainer.limelight;
    swerveDrive = RobotContainer.swerveDrive;
    remote = RobotContainer.swerveController;
    hood = RobotContainer.hood;
    inner = RobotContainer.innerIndex;
    this.shooter = RobotContainer.shooter;
    outer = RobotContainer.outerIndex;
    this.shooter = RobotContainer.shooter;
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // swerveDrive.setPID(p, i, d);
    double mode = NetworkTableInstance.getDefault().getTable("/datatable").getEntry("shooterMode").getDouble(0);
    // NetworkTableInstance.getDefault().getTable("/limelight-sam").getEntry("ledMode").setDouble(0);
    System.out.println("Aligning left and right");
    double startTime = 0;
    double turnStartTime = 0;
    System.out.println(light.hasTarget());
    if (mode == 0) {
      if (light.hasTarget() == 1) {
        System.out.println("has target");
        double dist = 0;
        if (light.getXDistance() <= 9) {
          System.out.println("Short Distance");
          dist = 1;
          shooter.set(3.45);
          startTime = System.currentTimeMillis();
        } else if (light.getXDistance() <= 12) {
          System.out.println("Medium Distance");
          dist = 2;
          shooter.set(3.65);//3.71
          startTime = System.currentTimeMillis();
        } else {
          System.out.println("Long Distance");
          dist = 3;
          shooter.set(4.15);
          startTime = System.currentTimeMillis();
        }
        double offset = light.getXOffset();
        turnStartTime = System.currentTimeMillis();
        System.out.println(Math.abs(offset) + "1");
        while (Math.abs(offset) > 2.5) {
          System.out.println(Math.abs(offset) + "2");
          if (offset < 0) {
            swerveDrive.updatePeriodic(0, 0, -0.060 * Math.sqrt(Math.abs(offset)));
            System.out.println(Math.abs(offset)+ "3");


      //       System.out.println("turning");

          } else {
            swerveDrive.updatePeriodic(0, 0, 0.060 * Math.sqrt(Math.abs(offset)));
          }
          offset = light.getXOffset();
        }
        swerveDrive.stopAll();
        System.out.println("Finished turning");
        hood.adjustAngle(light.getXDistance());
        System.out.println("Setting Hood to : " + light.getXDistance());
        System.out.println(Math.abs(offset));
      } else {

        System.out.println("Manual, can't find target");
        shooter.set(3.5);
        hood.adjustAngle(15);
        startTime = System.currentTimeMillis();
        turnStartTime = System.currentTimeMillis();

    } else {
      shooter.set(3.5);
      hood.adjustAngle(15);
      startTime = System.currentTimeMillis();
      turnStartTime = System.currentTimeMillis();

    }
    // finished starting up the flywheel and autoaiming left and right and setting
    // hood angle
    // wait for flywheel to ramp up
    System.out.println("Current Time waiting: " + System.currentTimeMillis());
    while (Math.abs(startTime - System.currentTimeMillis()) < 1000
        && Math.abs(turnStartTime - System.currentTimeMillis()) < 1000) {
      continue;
    }
    System.out.println("Finished first waiting: " + System.currentTimeMillis());

    double midStart = System.currentTimeMillis();
    System.out.println("loop 2");
    inner.spin();
    outer.spin();

    while (Math.abs(midStart - System.currentTimeMillis()) < 1500) {
      continue;
    }
    System.out.println("Finished loop 2");
    shooter.set(0);
    inner.stop();
    outer.stop();
    RobotContainer.status = 1;
  }

  @Override
  public void end(boolean interrupted) {
    // NetworkTableInstance.getDefault().getTable("/limelight-sam").getEntry("ledMode").setDouble(1);
    NetworkTableInstance.getDefault().getTable("/datatable").getEntry("SwerveCommand").setBoolean(false);
    // swerveDrive.stopAll();
    shooter.set(0);
    inner.stop();
    outer.stop();
  }

  @Override
  public boolean isFinished() {
    if (RobotContainer.status == 1) {
      RobotContainer.status = 0;
      return true;
    }
    return false;
  }
}