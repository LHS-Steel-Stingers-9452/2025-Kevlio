
package frc.robot.subsystems.intake;

import java.io.ObjectInputFilter.Config;
import java.lang.module.Configuration;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

 
public class Intake extends SubsystemBase {
 
    
    private final TalonFX  intakeKraken = new TalonFX(13);

    public final CANrange sensor = new CANrange(19);
    
    public Intake() {

        var motorOutputConfig =
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        var currentLimitConfig =
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(100)
                .withSupplyCurrentLimit(50)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true);

        var talonFXConfig = 
            new TalonFXConfiguration()
            .withCurrentLimits(currentLimitConfig)
            .withMotorOutput(motorOutputConfig);
            
        
        intakeKraken.getConfigurator().apply(talonFXConfig);
    }
   

    public void stopIntake() {
        intakeKraken.set(0);
    }

    public Command runIntake(double speed){
        return run(() -> {
            intakeKraken.set(speed);
        });
    }

    public Command intakeAlgae() {
        return runIntake(0.5);
    }

    @Logged
    public double getRange() {
        return sensor.getDistance().getValueAsDouble();
    }

    @Logged
    public boolean hasCoral() {

      return getRange()< 0.15;
    
    }
    @Logged
    public double intakeVoltage() {
        return intakeKraken.getMotorVoltage().getValueAsDouble();
    }
    
    @Logged
    public double intakePosition() {
        return intakeKraken.getPosition().getValueAsDouble();
    }
    
    @Logged
    public double intakeSupplyCurrent() {
        return intakeKraken.getSupplyCurrent().getValueAsDouble();
    }
    
    @Logged
    public double intakeStatorCurrent() {
        return intakeKraken.getStatorCurrent().getValueAsDouble();
    }
    
    @Logged
    public double intakeVelocity() {
        return intakeKraken.getVelocity().getValueAsDouble();
    }
    
    
} 

 

    
