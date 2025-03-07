package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.FunnelPivot;
import frc.robot.subsystems.intake.Intake;

public class CommandManager {

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
    

}

public static Command defaultElevator(Elevator elevator){
    return elevator.setPosition(0.05);
  //  .until(()-> elevator.elevatorPosition() <0.2)
    //.andThen(elevator.runElevator(0).withTimeout(0.5));
}

public static Command netPosition(Elevator elevator, Arm arm){
    return elevator.setPosition(5.3)
    .until(()-> elevator.elevatorPosition() > 5.2)
    .andThen(
        elevator.setPosition(6)
     .alongWith(arm.setPosition(0.27))
     
     );  

}
public static Command climbPose(FunnelPivot funnelPivot, Arm arm){
    return funnelPivot.runPivot(-0.34)
    .withTimeout(0.6)
    .andThen(funnelPivot.runPivot(0))
    .alongWith(arm.setPosition(0.30));
}

public static Command climberClimb(Climber climber){
    return climber.runClimber(0.1);
    // .finallyDo(climber.stopClimber())

}

}
