package frc.robot.subsystems.funnel;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class FunnelPivot extends SubsystemBase {

    
    private final TalonFX  pivotKraken = new TalonFX(9);
    
    public FunnelPivot() {

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
            
        
        pivotKraken.getConfigurator().apply(talonFXConfig);
    }


   

    public Command stopPivot() {
        return runOnce(() -> {
            pivotKraken.set(0);
        });
        }

    /* public Command tryPosition(){
        return runOnce(() -> {
            pivotKraken.setPosition(-5.0);
        });
    }
    */
    
    public Command runPivot(double speed){
        return run(() -> {
            pivotKraken.set(speed);
        });
        }

    @Logged    
    public double pivotPosition() {
    return pivotKraken.getPosition().getValueAsDouble();
    }

}