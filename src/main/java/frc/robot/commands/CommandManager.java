package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.FunnelPivot;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class CommandManager {
    //  work in progress (desperately needs some un-.andThen'ining)
  /*   public static Command coralL4(Arm arm, Elevator elevator,Intake intake){

            return setPositions(arm, elevator, 0.23, 5.2) //All hail the wall of .andThen
            .andThen(new WaitCommand(3)) //adjust the wait time accordingly
            .andThen(intake.runIntake(-0.6))
            .andThen(new WaitCommand(2.5))
            .andThen(defaultElevator(elevator))
            .andThen(defaultArm(arm));

    }
    */
    public static Command intakeCoral(Funnel funnel, Intake intake){

            //auto intake coral
            return funnel.runFunnel(0.4).alongWith(intake.runIntake(0.2))
                    .until(intake::hasCoral)
                    .andThen(new WaitCommand(0.10))
                    .andThen(funnel.stopFunnel().alongWith(intake.stopIntake()));
        }

    public static Command setPositions(Arm arm, Elevator elevator, double armPosition, double elevatorPosition){
             return arm.setPosition(0.25)
             .until(()-> arm.armPosition() < 0.27 && arm.armPosition() > 0.23)
             .andThen(
                elevator.setPosition(elevatorPosition)
             .alongWith(arm.setPosition(armPosition))
             
             );   
    }

    public static Command intakePositions(Arm arm, Elevator elevator) {
    // return arm.setPosition(0.36).alongWith(elevator.runElevator(0));
    // return arm.runArm(0.1).alongWith(elevator.runElevator(0)).withTimeout(0.5);
        return arm.runArm(0.1).withTimeout(0.5).andThen(elevator.runElevator(0)).andThen(arm.runArm(0));
    }

    public static Command defaultPositions(Arm arm, Elevator elevator, Intake intake) {
        return setPositions(arm, elevator, 0.30, 0.15)
        .alongWith(intake.runIntake(0));
    }

    public static Command defaultArm(Arm arm){
        return arm.runArm(0.1)
        .until(()-> arm.armPosition() < 0.37 && arm.armPosition() > 0.36)
        .andThen(arm.runArm(0));

        
       // return arm.setPosition(0.34); //0.365
    }

public static Command defaultElevator(Elevator elevator){
    return elevator.setPosition(0.02);
  //  .until(()-> elevator.elevatorPosition() <0.2)
    //.andThen(elevator.runElevator(0).withTimeout(0.5));
}

    public static Command netPosition(Elevator elevator, Arm arm){
        return elevator.setPosition(5.8)
        .alongWith(arm.setPosition(0.16))
        .until(()-> elevator.elevatorPosition() > 5.8)
        .andThen(
            elevator.setPosition(6)
        .alongWith(arm.setPosition(0.225))
        
        );  

}

public static Command climbPose(FunnelPivot funnelPivot, Arm arm){
    return funnelPivot.runPivot(-0.34)
    .withTimeout(0.44)
    .andThen(funnelPivot.runPivot(0))
    .alongWith(arm.setPosition(0.30));
}

    public static Command climberClimb(Climber climber){
        return climber.runClimber(0.1);
        // .finallyDo(climber.stopClimber())

    }
   

//  autoCommands

    public static Command L4Pose(Elevator elevator, Arm arm){
        return setPositions(arm, elevator, 0.213, 5.3);
    }

  //  public static Command scoreCoral(Intake intake){
   //     return 

    //}

    public static Command scoreL4(Elevator elevator, Arm arm, Intake intake){
        return setPositions(arm, elevator, 0.213, 5.3)
        .until(()-> elevator.elevatorPosition() > 5.25)
        .andThen((intake.runIntake(1)
       .withTimeout(4)));
   }



   public static Command testElevator(Elevator elevator){
  return new SequentialCommandGroup(new InstantCommand(() -> elevator.testPosition(5.3)));



   }

   public static Command defaultPoses(Elevator elevator, Arm arm){
    return 
    (setPositions(arm, elevator, 0.25,0.045))
    .andThen(defaultArm(arm));

   }

}
