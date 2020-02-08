#include <frc/WPILib.h>
#include <math.h>

#include "ShooterSubsystem.h"
#include "RobotConstants.h"

const double c_rpmTolerance = 200.0;

ShooterSubsystem::ShooterSubsystem(EntechRobot *pRobot, std::string name)
    : RobotSubsystem(pRobot, name)
    , m_ShooterMotor(NULL)
    , m_solenoid1(NULL)
    , m_solenoid2(NULL)
    , m_mode(kVbus)
    , m_shoot(true)
    , m_speed(0.0)
    , m_rpm(0.0)
    , m_pidF(1.55)
    , m_pidP(22.0)
    , m_pidI(0.0)
    , m_pidD(800.0)
{
}

ShooterSubsystem::~ShooterSubsystem()
{
}

void ShooterSubsystem::Forward(double speed)
{
    m_speed = speed;
    m_mode = kVbus;
    m_ShooterMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, m_speed);
}

void ShooterSubsystem::SetRPM(double rpm)
{
    m_rpm = rpm;
    m_mode = kRPM;
    m_ShooterMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, m_rpm);
}

bool ShooterSubsystem::IsAtTargetRPM(void)
{
    // if (fabs(m_ShooterMotor->GetSpeed() - m_rpm) < c_rpmTolerance) {
    if (m_ShooterMotor->GetSelectedSensorVelocity(0) > (m_rpm-c_rpmTolerance)) {
        return true;
    }
    return false;
}

void ShooterSubsystem::TriggerOpen(void)
{
    m_shoot = false;
}

void ShooterSubsystem::TriggerClose(void)
{
    m_shoot = true;
}

void ShooterSubsystem::RobotInit()
{
    m_ShooterMotor = new TalonSRX(c_ShooterMotor_CANid);
    m_ShooterMotor->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder,0,0);
    m_ShooterMotor->ConfigNominalOutputForward(0.0,0);
    m_ShooterMotor->ConfigNominalOutputReverse(0.0,0);
    m_ShooterMotor->ConfigPeakOutputForward(11.5,0);
    // TODO: Find replacement for next line
    // m_ShooterMotor->ConfigEncoderCodesPerRev(20);  // actual  encoder docs say 20 pulses per channel

    // functions to invert the motor or sensor
    // m_ShooterMotor->SetInverted(true);
    // m_ShooterMotor->SetSensorDirection(true);

    SetPIDController();

    //m_ShooterMotor->SetVelocityMeasurementPeriod(CANTalon::Period_10Ms);
    //m_ShooterMotor->SetVelocityMeasurementWindow(64);
    //m_ShooterMotor->SetControlMode(CANSpeedController::kPercentVbus);

    m_solenoid1 = new frc::Solenoid(c_compressorPCMid, c_shooterSolenoidChannel1);
    m_solenoid2 = new frc::Solenoid(c_compressorPCMid, c_shooterSolenoidChannel2);
}

void ShooterSubsystem::SetPIDController(void)
{
    frc::Preferences *prefs = frc::Preferences::GetInstance();
    m_pidF = prefs->GetDouble("shooterF", 1.55);
    m_pidP = prefs->GetDouble("shooterP", 22.0);
    m_pidI = prefs->GetDouble("shooterI", 0.0);
    m_pidD = prefs->GetDouble("shooterD", 800.0);
    m_ShooterMotor->Config_kF(0,m_pidF,0);
    m_ShooterMotor->Config_kP(0,m_pidP,0);
    m_ShooterMotor->Config_kI(0,m_pidI,0);
    m_ShooterMotor->Config_kD(0,m_pidD,0);
    m_ShooterMotor->SelectProfileSlot(0,0);
    m_ShooterMotor->ConfigAllowableClosedloopError(0,0,0);
}

void ShooterSubsystem::UpdateDashboard()
{
    frc::SmartDashboard::PutNumber("Shooter JS Speed", m_speed);
    frc::SmartDashboard::PutNumber("Shooter Sensor Speed", m_ShooterMotor->GetSelectedSensorVelocity(0));
}

void ShooterSubsystem::LogHeader(FILE *fp)
{
    fputs("ShooterRPM,Shooting,",fp);
}

void ShooterSubsystem::LogData(FILE *fp)
{
    fprintf(fp,"%d,%d,",m_ShooterMotor->GetSelectedSensorVelocity(0),m_shoot);
}

void ShooterSubsystem::TeleopInit()
{
    SetPIDController();
    m_shoot = true;
}

void ShooterSubsystem::AutonomousInit()
{
    m_shoot = true;
}

void ShooterSubsystem::TestInit()
{
}

void ShooterSubsystem::DisabledInit()
{
    m_shoot = true;
}

void ShooterSubsystem::DisabledPeriodic()
{
    m_ShooterMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
}

void ShooterSubsystem::TeleopPeriodic()
{
    AutonomousPeriodic();
}

void ShooterSubsystem::AutonomousPeriodic()
{
    switch (m_mode) {
    case kVbus:
        frc::SmartDashboard::PutString("Shooter Mode", "PercentVbus");
        frc::SmartDashboard::PutNumber("Shooter Speed", m_speed);
        m_ShooterMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, m_speed);
        break;
    case kRPM:
        frc::SmartDashboard::PutString("Shooter Mode", "SpeedMode");
        m_ShooterMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, m_rpm);
        break;
    }
    if (m_shoot) {
        m_solenoid1->Set(true);
        m_solenoid2->Set(false);
    } else {
        m_solenoid1->Set(false);
        m_solenoid2->Set(true);
    }
}

void ShooterSubsystem::TestPeriodic()
{
}
