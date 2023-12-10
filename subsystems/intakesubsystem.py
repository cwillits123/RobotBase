from enum import Enum, auto
from commands2 import SubsystemBase
from wpilib import PneumaticsModuleType, Solenoid
from ctre import ControlMode
from util.ctrecheck import ctreCheckError
from util.simfalcon import createMotor
import constants


class IntakeSubsystem(SubsystemBase):
    class IntakeMode(Enum):
        Deployed = auto()  #Deployed
        Neutral = auto()  #Retracted or Neutral
        Reversed = auto()  #Reversed

    def __init__(self):
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)

        self.intakeMotor = createMotor(  #Creating the intake Motor
            constants.kIntakeMotorId,
            constants.kCANivoreName,
        )

        self.intakeSolenoidMag = Solenoid(  #Creating the solenoid
            PneumaticsModuleType.REVPH, constants.kIntakeSolenoidId
        )

        print(f"Initalizing IntakeMotor")
        if not ctreCheckError(  #Checking for errors when setting up
            "configFactoryDefault",
            self.intakeMotor.configFactoryDefault(constants.kConfigurationTimeoutLimit),
        ):
            return
        if not ctreCheckError(
            "config_kP",
            self.intakeMotor.config_kP(
                constants.kIntakePIDSlot,
                constants.kIntakePGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_kI",
            self.intakeMotor.config_kI(
                constants.kIntakePIDSlot,
                constants.kIntakeIGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_kD",
            self.intakeMotor.config_kD(
                constants.kIntakePIDSlot,
                constants.kIntakeDGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        print(f"IntakeMotor Initialization Complete")
        self.state = self.IntakeMode.Neutral  #Neutral motor position and initial start up

    def period(self) -> None:
        if self.state == self.IntakeMode.Deployed:  #When delpoyed, setting Solenoid to true and setting speeds
            self.intakeSolenoidMag.set(True)
            self.intakeMotor.set(
                ControlMode.Velocity,
                constants.kIntakeMotorSpeed
                * constants.kIntakeGearRatio
                * constants.kTalonVelocityPerRPM,
            )
        elif self.state == self.IntakeMode.Reversed:  #When reversed, doing same as deployed yet in negative velocity
            self.intakeSolenoidMag.set(True)
            self.intakeMotor.set(
                ControlMode.Velocity,
                -constants.kIntakeMotorSpeed
                * constants.kIntakeGearRatio
                * constants.kTalonVelocityPerRPM,
            )
        elif self.state == self.IntakeMode.Neutral:  #Setting the motor to neutral
            self.intakeSolenoidMag.set(False)
            self.intakeMotor.set(
                ControlMode.Velocity, 0
            )

    def reversedIntake(self) -> None:
        self.state = self.IntakeMode.Reversed
    
    def neutralIntake(self) -> None:
        self.state = self.IntakeMode.Neutral
    
    def deployedIntake(self) -> None:
        self.state = self.IntakeMode.Deployed
