package frc.robot.subsystems.funnel;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase{


      private final TalonFX  funnelKraken = new TalonFX(14);

    public Funnel() {

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
            
        
        funnelKraken.getConfigurator().apply(talonFXConfig);
    }
   

    public Command stopFunnel() {
     return runOnce(() -> {
        funnelKraken.set(0);
    });
    }

    public Command runFunnel(double speed){
        return run(() -> {
            funnelKraken.set(speed);
        });
    }
}