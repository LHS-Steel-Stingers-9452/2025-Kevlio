// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.CommandManager;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.FunnelPivot;
import frc.robot.subsystems.intake.Intake;

@Logged
public class RobotContainer {
  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts
          .baseUnitMagnitude(); // kSpeedAt12Volts desired top speed, tune later
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity


  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


  //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
 // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

  private final Telemetry logger = new Telemetry(MaxSpeed);


  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);



  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  //public final Elevator elevator = new Elevator();
  public final Elevator elevator;

  //public final Arm arm = new Arm();
  public final Arm arm;

  //public final Intake intake = new Intake();
  public final Intake intake;

  //public final Climber climber = new Climber();
  public final Climber climber;

 // public final Funnel funnel = new Funnel();
  public final Funnel funnel;

  //public final FunnelPivot funnelPivot = new FunnelPivot();
  public final FunnelPivot funnelPivot;


  private final SwerveRequest.RobotCentric robotRelativeDrive =
      new SwerveRequest.RobotCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


   private final SendableChooser<Command> autoChooser;




  public RobotContainer() {

    elevator = new Elevator();

    arm = new Arm();

    intake = new Intake();

    climber = new Climber();

    funnel = new Funnel();

    funnelPivot = new FunnelPivot();

    configureBindings();

    //register named commands
     NamedCommands.registerCommand("scoreL4", CommandManager.scoreL4(elevator, arm, intake));
     NamedCommands.registerCommand("defaultPoses", CommandManager.defaultPoses(elevator, arm));
     NamedCommands.registerCommand("setL4Pose", CommandManager.L4Pose(elevator, arm));
     NamedCommands.registerCommand("print", Commands.runOnce(()-> System.out.println("commandsent")));


    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
     autoChooser = AutoBuilder.buildAutoChooser();
     
     SmartDashboard.putData("Auto Chooser", autoChooser);
  }


  private void configureBindings() {
/* 
    SmartDashboard.putData("runArm", arm.runArm(0.1));
    SmartDashboard.putData("runArmBackwards", arm.runArm(-0.1));
    SmartDashboard.putData("stopArm", arm.runArm(0));
    SmartDashboard.putData("zeroArmEncoder", arm.zeroArm());
    SmartDashboard.putData("setArmPose", arm.setPosition(0.5));

    SmartDashboard.putData("setEncoderStowPosition", arm.setArmEncoderStow());
    SmartDashboard.putData("go to zero motion magic", arm.setPosition(0));

    SmartDashboard.putData("runIntake", intake.runIntake(0.2));
    SmartDashboard.putData("runIntakeBackwards", intake.runIntake(-0.2));
  

    SmartDashboard.putData("runClimber", climber.runClimber(0.1));
    SmartDashboard.putData("runClimberBackwards", climber.runClimber(-1));

    SmartDashboard.putData("runFunnel", funnel.runFunnel(0.5));

    SmartDashboard.putData("runElevator", elevator.runElevator(0.1));
    SmartDashboard.putData("zeroElevatorEncoder", elevator.zeroElevatorEncoder());
    SmartDashboard.putData("setElevatorPosition 2.0", elevator.setPosition(2.0));
    SmartDashboard.putData("setElevatorPosition 3.0", elevator.setPosition(3.0));
   

    SmartDashboard.putData("algae L3", CommandManager.setPositions(arm, elevator, -0.16 , 2.8));
    SmartDashboard.putData("intakeAlgae", intake.runIntake(-0.2));
*/


    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.



//driver bindings
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -joystick.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

        

   /*  joystick
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));  
*/
    // joystick.b().whileTrue(elevator.sysIdDynamic(Direction.kForward));
    // joystick.x().whileTrue(elevator.sysIdDynamic(Direction.kReverse));
    // joystick.y().whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
    // joystick.a().whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));return 

      // reset the field-centric heading on right bumper press // changed 2/25 from left bumper to right bumper
    joystick.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));








    //TESTING FOR EDDIE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    joystick.rightTrigger().whileTrue(intake.runIntake(1));






    //low algae pose
    joystick.a().onTrue(CommandManager.setPositions(arm, elevator, -0.125, 1.8));

    // high algae pose
    joystick.y().onTrue(CommandManager.setPositions(arm, elevator, -0.125, 3.0));

    //processor pose
    joystick.x().onTrue(CommandManager.setPositions(arm, elevator, -0.125, 0.3));

    //processor stow pose (don't need BUT ynk)
    //joystick.b().onTrue(CommandManager.setPositions(arm, elevator, 0.1, 0.1));

    //stop intake
    //joystick.povLeft().onTrue(intake.runIntake(0));

    //climb up 
    joystick.povUp().whileTrue(climber.runClimber(1));

    //climb down
    joystick.povDown().whileTrue(climber.runClimber(-1));



//operator bindings

    //auto coral intake
    operator.povDown().onTrue(CommandManager.intakeCoral(funnel, intake));

    // move pivot funnel and arm for climb 
    operator.povRight().onTrue(CommandManager.climbPose(funnelPivot, arm));
    
    //score net
    operator.povUp().onTrue(CommandManager.netPosition(elevator, arm));

    // algae intake| coral outtake
    operator.rightTrigger().whileTrue(intake.runIntake(-0.6));

    //algae outtake
    operator.rightBumper().whileTrue(intake.runIntake(1));
    

    // score L2
    operator.x().onTrue(CommandManager.setPositions(arm, elevator, 0.34 , 1.15));
    // score L3
    operator.y().onTrue(CommandManager.setPositions(arm, elevator, 0.30 , 2.7));
    
    // Score L4
    operator.b().onTrue(CommandManager.setPositions(arm, elevator, 0.213 , 5.3)); //0.23 5.1
    
    // default arm (intake position)
    operator.leftBumper().onTrue(CommandManager.defaultArm(arm));

    //default elevator w/ arm out of the way
   operator.leftTrigger().onTrue(CommandManager.setPositions(arm, elevator, 0.25, 0.045));
    



    // joystick.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    //joystick.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
    


    drivetrain.registerTelemetry(logger::telemeterize);


   
//slow button that DOES WORK (in theory), driver
    joystick
        .leftBumper()
        .whileTrue(
            drivetrain.applyRequest(
                () -> 
                drive
                .withVelocityX(
                   (-joystick.getLeftY() * MaxSpeed)*(.25)) 
                .withVelocityY(
                    (-joystick.getLeftX() * MaxSpeed)*(.25))
                .withRotationalRate(
                    -joystick.getRightX()
                        *MaxAngularRate)
                                  
                ));

// vision bindings, for driver( he doesn't even use it T-T )

    //lock onto april tag
 /*        joystick
            .rightTrigger()
            .whileTrue(
                drivetrain.applyRequest(
                    () -> {
                    double rotation = limelight_aim_proportional();
                    double xSpeed = limelight_range_proportional();

                    SmartDashboard.putNumber("xSpeed", xSpeed);
                    SmartDashboard.putNumber("rotation", rotation);

                    return robotRelativeDrive
                        .withVelocityX(
                            -joystick.getLeftY()
                                * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(
                            -joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(
                            rotation); // Drive counterclockwise with negative X (left)
                    }));
                    */
                }


  public void autoInit(){
    drivetrain.seedFieldCentric();
    arm.defaultArmEncoder();
    elevator.defaultElevatorEncoder();
  }
  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the
  // "tx" value from the Limelight.
  double limelight_aim_proportional() {
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control
    // loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .0176;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= MaxAngularRate;

    // invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  double limelight_range_proportional() {
    // double kP = .1;
    double kP = .05;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= MaxSpeed;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

 /*  public Command getTaxiAuto() {
    return 
    drivetrain.applyRequest(() ->
      drive.withVelocityX(0.7)
      .withVelocityY(0)
     .withRotationalRate(0))
      .withTimeout(3)
      .andThen(drivetrain.applyRequest(() -> idle));
  }
 */

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

 // double limelightMoveForeward() {
 //   double tagID = LimelightHelpers.getFiducialID("limelight");
 //   return tagID;

 // }
}