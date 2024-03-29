package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Potentiometer;

public class ArmCommand extends CommandBase {
    
    Arm arm = new Arm();
    Potentiometer pot = new Potentiometer();
    Supplier<Double> raise;
    double targetPos;
    double kDt;
    TrapezoidProfile.Constraints m_constraints;
    ProfiledPIDController m_controller;

    public ArmCommand(Arm arm, Potentiometer pot, Supplier<Double> raise) {
        this.arm = arm;
        this.pot = pot;
        this.raise = raise;
        this.targetPos = 90;
        this.kDt = 0.02;
        m_constraints = new TrapezoidProfile.Constraints(20, 5);
        m_controller = new ProfiledPIDController(0.025, 0.04, 0.00, m_constraints, kDt);
        addRequirements(arm);
    }

    public void initialize() {
    }

    @Override 
    public void execute(){
        m_controller.setGoal(90);
        double curPos = -1*Potentiometer.readPot();
        arm.manualArm(m_controller, curPos);
    }
@Override 
public void end(boolean interrupted){
    m_controller.setGoal(0);
    arm.setVal(0);
}

@Override
public boolean isFinished(){
    return Math.abs(Potentiometer.readPot() + m_controller.getGoal().position) < 2;

}   

    
}