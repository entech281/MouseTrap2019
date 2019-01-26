#pragma once

#include <frc/WPILib.h>
#include <list>
#include <map>

#include "RobotSubsystem.h"

#include "RobotConstants.h"
#include "DriveSubsystem.h"
#include "ClimberSubsystem.h"
#include "DropperSubsystem.h"
#include "PickUpSubsystem.h"
#include "ShooterSubsystem.h"
#include "OperatorButton.h"

class EntechRobot : public frc::TimedRobot {
public:
    EntechRobot();
    virtual ~EntechRobot();

    void RegisterSubsystem(RobotSubsystem*);
    bool IsGearDropped(void);
    bool IsPinSensed(void);
    bool IsInAutoDropMode(void);
    double InitialYaw(void);
    void ZeroInitialYaw(void);
    inline DriveSubsystem *GetDriveSubsystem(void) { return m_drive; }
    inline DropperSubsystem *GetDropperSubsystem(void) {return m_dropper; }

protected:
    void UpdateDashboard();
    void OpenLog(void);
    void WriteLog(void);
    void CloseLog(void);

    virtual void RobotInit();
    virtual void DisabledInit();
    virtual void DisabledPeriodic();
    virtual void TeleopInit();
    virtual void TeleopPeriodic();
    virtual void AutonomousInit();
    virtual void AutonomousPeriodic();
    virtual void TestInit();
    virtual void TestPeriodic();

private:
    void DetermineAutonomousSetup(void);
    void ResetGearDropAction(void);
    void DoGearDropAction(void);
    void StartGearDropAction(void);
    DriveSubsystem   *m_drive;
    ClimberSubsystem *m_climber;
    ShooterSubsystem *m_shooter;
    DropperSubsystem *m_dropper;
    PickUpSubsystem  *m_pickup;
    Compressor *m_compressor;
    // LiveWindow *m_lw;

    FILE *m_logFP;

    frc::Joystick  *m_gamepad;
    OperatorButton *m_gp_useShooterPID;
    OperatorButton *m_gp_climbButton;
    OperatorButton *m_gp_descendButton;
    OperatorButton *m_gp_dropButton;
    OperatorButton *m_gp_pickupButton;
    OperatorButton *m_gp_autodropButton;
    OperatorButton *m_gp_fireButton;

    frc::Joystick *m_buttonpanel;
    OperatorButton *m_bp_climbButton;
    OperatorButton *m_bp_dropButton;
    OperatorButton *m_bp_autodropButton;
    OperatorButton *m_bp_shooterOnButton;
    OperatorButton *m_bp_fireButton;
    OperatorButton *m_bp_yawLeftButton;
    OperatorButton *m_bp_yawRightButton;
    OperatorButton *m_bp_yawZeroButton;

    std::list<RobotSubsystem*> m_robotSubsystems;

    enum AutoDropState { kDropStart, kDropWaitForGearRelease,
                         kDropBackoff, kWaitForDropBackoff,
                         kDropRaise, kWaitForDropRaise,
                         kDropForward, kWaitForDropForward,
                         kDropBackup, kWaitForDropBackup, kDropDone
   };
    AutoDropState m_dropState;
    frc::Timer *m_dropTimer;

    bool m_autonomousActive;
    frc::DigitalInput *m_autoSelectionD1;
    frc::DigitalInput *m_autoSelectionD2;
    frc::DigitalInput *m_autoSelectionD3;
    enum AutoState { kStart = 0,
                     kTurnOnShooter, kWaitForShooterToSpinup,
                     kShootFuelLoad, kWaitForShootFuelLoad,
                     kInitialDrive, kWaitForInitialDrive,
                     kInitialTurn, kWaitForInitialTurn,
                     kDriveToTarget, kWaitForDriveToTarget,
                     kDriveBackward, kWaitForDriveBackward,
                     kDriveLateral, kWaitForDriveLateral,
                     kDriveForward, kWaitForDriveForward,
                     kSetSideShotYaw, kWaitForSetSideShotYaw,
                     kClearAirship, kWaitForClearAirship,
                     kAlignToTarget, kWaitForAlignToTarget,
                     kBackupToEndWall, kWaitForBackupToEndWall,
                     kBackupToWall, kWaitForBackupToWall,
                     kNudgeBack, kWaitForNudgeBack,
                     kNudgeSide, kWaitForNudgeSide,
                     kDone };
    AutoState m_autoState;
    bool m_autoSecondTry;
    enum AutoNudgeDirection { kLeft, kRight };
    int m_autoNudgeCount;
    AutoNudgeDirection m_autoNudgeDir;
    enum BoilerDistance { kNear, kMiddle, kFar, kSiderail };
    BoilerDistance m_boilerDistance;
    enum InitialTurn { kRight60, kStraight, kLeft60 };
    InitialTurn m_initialTurn;
    bool m_boilerToLeft;
    bool m_yawZeroed;
    frc::Timer *m_autoTimer;

    frc::Preferences *m_prefs;
    double m_shooterSpeed;
    double m_shooterTime;
};
