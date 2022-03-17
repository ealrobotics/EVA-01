// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSS extends SubsystemBase {
    // The plant holds a state-space model of our flywheel. This system has the
    // following properties:
    //
    // States: [velocity], in radians per second.
    // Inputs (what we can "put in"): [voltage], in volts.
    // Outputs (what we can measure): [velocity], in radians per second.
    //
    // The Kv and Ka constants are found using the FRC Characterization toolsuite.
    private final LinearSystem<N1, N1, N1> m_BflywheelPlant = LinearSystemId.identifyVelocitySystem(
            ShooterConstants.kBottomFlywheelKv,
            ShooterConstants.kBottomFlywheelKa);
    /*
     * private final LinearSystem<N1, N1, N1> m_TflywheelPlant =
     * LinearSystemId.identifyVelocitySystem(
     * ShooterConstants.kTopFlywheelKv,
     * ShooterConstants.kTopFlywheelKa);
     */

    // The observer fuses our encoder data and voltage inputs to reject noise.
    private final KalmanFilter<N1, N1, N1> m_Bobserver = new KalmanFilter<>(
            Nat.N1(),
            Nat.N1(),
            m_BflywheelPlant,
            VecBuilder.fill(3.0), // How accurate we think our model is
            VecBuilder.fill(0.01), // How accurate we think our encoder
            // data is
            0.020);
    /*
     * private final KalmanFilter<N1, N1, N1> m_Tobserver = new KalmanFilter<>(
     * Nat.N1(),
     * Nat.N1(),
     * m_TflywheelPlant,
     * VecBuilder.fill(3.0), // How accurate we think our model is
     * VecBuilder.fill(0.01), // How accurate we think our encoder
     * // data is
     * 0.020);
     */

    // A LQR uses feedback to create voltage commands.
    private final LinearQuadraticRegulator<N1, N1, N1> m_Bcontroller = new LinearQuadraticRegulator<>(
            m_BflywheelPlant,
            VecBuilder.fill(8.0), // Velocity error tolerance
            VecBuilder.fill(12.0), // Control effort (voltage) tolerance
            0.020);
    /*
     * private final LinearQuadraticRegulator<N1, N1, N1> m_Tcontroller = new
     * LinearQuadraticRegulator<>(
     * m_TflywheelPlant,
     * VecBuilder.fill(8.0), // Velocity error tolerance
     * VecBuilder.fill(12.0), // Control effort (voltage) tolerance
     * 0.020);
     */

    // The state-space loop combines a controller, observer, feedforward and plant
    // for easy control.
    private final LinearSystemLoop<N1, N1, N1> m_Bloop = new LinearSystemLoop<>(m_BflywheelPlant, m_Bcontroller,
            m_Bobserver,
            12.0, 0.020);
    /*
     * private final LinearSystemLoop<N1, N1, N1> m_Tloop = new
     * LinearSystemLoop<>(m_TflywheelPlant, m_Tcontroller,
     * m_Tobserver,
     * 12.0, 0.020);
     */

    // An encoder set up to measure flywheel velocity in radians per second.
    private final Encoder m_bottomEncoder = new Encoder(ShooterConstants.kBottomEncoderPorts[0],
            ShooterConstants.kBottomEncoderPorts[1]);
    /*
     * private final Encoder m_topEncoder = new
     * Encoder(ShooterConstants.kTopEncoderPorts[0],
     * ShooterConstants.kTopEncoderPorts[1]);
     */

    // Shooter motors
    private final CANSparkMax m_bottomMotor = new CANSparkMax(CANIDConstants.shooterBottomMotorID,
            MotorType.kBrushless);
    /*
     * private final CANSparkMax m_topMotor = new
     * CANSparkMax(CANIDConstants.shooterTopMotorID, MotorType.kBrushed);
     */

    /** Creates a new ShooterState. */
    public ShooterSS() {
        // We go 2 pi radians per 4096 clicks.
        m_bottomEncoder.setDistancePerPulse(ShooterConstants.kBottomEncoderDistancePerPulse);
        /*
         * m_topEncoder.setDistancePerPulse(ShooterConstants.kTopEncoderDistancePerPulse
         * );
         */

        // Reset our loop to make sure it's in a known state.
        m_Bloop.reset(VecBuilder.fill(m_bottomEncoder.getRate()));
        /* m_Tloop.reset(VecBuilder.fill(m_topEncoder.getRate())); */
    }

    public void run(double bottomShooterSpeed) {
        // Sets the target speed of our flywheel. This is similar to setting the
        // setpoint of a
        // PID controller.
        double kBSpinupRadPerSec = Units
                .rotationsPerMinuteToRadiansPerSecond(bottomShooterSpeed);
        m_Bloop.setNextR(VecBuilder.fill(kBSpinupRadPerSec));
        /*
         * double kTSpinupRadPerSec = Units
         * .rotationsPerMinuteToRadiansPerSecond(topShooterSpeed);
         * m_Tloop.setNextR(VecBuilder.fill(kTSpinupRadPerSec));
         */

        // Correct our Kalman filter's state vector estimate with encoder data.
        m_Bloop.correct(VecBuilder.fill(m_bottomEncoder.getRate()));
        /* m_Tloop.correct(VecBuilder.fill(m_topEncoder.getRate())); */

        // Update our LQR to generate new voltage commands and use the voltages to
        // predict the next
        // state with out Kalman filter.
        m_Bloop.predict(0.020);
        /* m_Tloop.predict(0.020); */

        // Send the new calculated voltage to the motors.
        // voltage = duty cycle * battery voltage, so
        // duty cycle = voltage / battery voltage
        double BnextVoltage = m_Bloop.getU(0);
        m_bottomMotor.setVoltage(BnextVoltage);
        /*
         * double TnextVoltage = m_Tloop.getU(0);
         * m_topMotor.setVoltage(TnextVoltage);
         */
    }

    @Override
    public void periodic() {

    }
}
