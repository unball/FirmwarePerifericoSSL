#include "Driver.hpp"

namespace Driver
{   

    static MagneticSensorI2C encoder_out0 = MagneticSensorI2C(AS5600_I2C);
    static TwoWire I2CEncoder_out0 = TwoWire(parameters::I2C_BUS_NUMBER_0);
    static BLDCMotor motor_out0 = BLDCMotor(parameters::NUMBER_POLES, parameters::PHASE_RESISTANCE);
    static BLDCDriver3PWM driver_out0 = BLDCDriver3PWM(pins::DRIVER_PINOUT_PHASE_A_0, pins::DRIVER_PINOUT_PHASE_B_0, pins::DRIVER_PINOUT_PHASE_C_0, pins::DRIVER_PINOUT_ENABLE_0);
    static InlineCurrentSense currentSense_out0 = InlineCurrentSense(parameters::CURRENT_SENSE_RESISTOR, parameters::CURRENT_SENSE_GAIN, pins::CURRENT_SENSE_PINOUT_PHASE_A_0, pins::CURRENT_SENSE_PINOUT_PHASE_B_0);

    static MagneticSensorI2C encoder_out1 = MagneticSensorI2C(AS5600_I2C);
    static TwoWire I2CEncoder_out1 = TwoWire(parameters::I2C_BUS_NUMBER_1);
    static BLDCMotor motor_out1 = BLDCMotor(parameters::NUMBER_POLES, parameters::PHASE_RESISTANCE);
    static BLDCDriver3PWM driver_out1 = BLDCDriver3PWM(pins::DRIVER_PINOUT_PHASE_A_1, pins::DRIVER_PINOUT_PHASE_B_1, pins::DRIVER_PINOUT_PHASE_C_1, pins::DRIVER_PINOUT_ENABLE_1);
    static InlineCurrentSense currentSense_out1 = InlineCurrentSense(parameters::CURRENT_SENSE_RESISTOR, parameters::CURRENT_SENSE_GAIN, pins::CURRENT_SENSE_PINOUT_PHASE_A_1, pins::CURRENT_SENSE_PINOUT_PHASE_B_1);

    void setup(uint8_t controlType, boolean configureMotor0, boolean configureMotor1){

        Serial.begin(115200);
        SimpleFOCDebug::enable(&Serial);

        if(configureMotor0){

            executeMotor0 = configureMotor0;

            I2CEncoder_out0.begin(pins::I2C_PINOUT_SDA_0,pins::I2C_PINOUT_SCL_0, parameters::I2C_PINOUT_FREQUENCY);
            encoder_out0.init(&I2CEncoder_out0);
            motor_out0.linkSensor(&encoder_out0);

            driver_out0.voltage_power_supply = parameters::VOLTAGE_POWER_SUPPLY;
            driver_out0.voltage_limit = parameters::VOLTAGE_LIMIT;
            driver_out0.init();
            motor_out0.linkDriver(&driver_out0);
            currentSense_out0.linkDriver(&driver_out0);

            motor_out0.foc_modulation = FOCModulationType::SpaceVectorPWM;

            setupTypeControl(controlType, &motor_out0);

            motor_out0.current_limit = parameters::MOTOR_CURRENT_LIMIT;
            motor_out0.voltage_limit = parameters::MOTOR_VOLTAGE_LIMIT;
            motor_out0.velocity_limit = parameters::MOTOR_VELOCITY_LIMIT;

            motor_out0.linkCurrentSense(&currentSense_out0);
            currentSense_out0.gain_b *= parameters::CURRENT_SENSE_GAIN_B;
            currentSense_out0.init();

            motor_out0.init();
            motor_out0.initFOC();     
        }
        
        if(configureMotor1){

            executeMotor1 = configureMotor1;

            I2CEncoder_out1.begin(pins::I2C_PINOUT_SDA_1,pins::I2C_PINOUT_SCL_1, parameters::I2C_PINOUT_FREQUENCY);
            encoder_out1.init(&I2CEncoder_out1);
            motor_out1.linkSensor(&encoder_out1);

            driver_out1.voltage_power_supply = parameters::VOLTAGE_POWER_SUPPLY;
            driver_out1.voltage_limit = parameters::VOLTAGE_LIMIT;
            driver_out1.init();
            motor_out1.linkDriver(&driver_out1);
            currentSense_out1.linkDriver(&driver_out1);

            motor_out1.foc_modulation = FOCModulationType::SpaceVectorPWM;

            setupTypeControl(controlType, &motor_out1);

            motor_out1.current_limit = parameters::MOTOR_CURRENT_LIMIT;
            motor_out1.voltage_limit = parameters::MOTOR_VOLTAGE_LIMIT;
            motor_out1.velocity_limit = parameters::MOTOR_VELOCITY_LIMIT;

            motor_out1.linkCurrentSense(&currentSense_out1);
            currentSense_out1.gain_b *= parameters::CURRENT_SENSE_GAIN_B;
            currentSense_out1.init();

            motor_out1.init();
            motor_out1.initFOC();
        }
    }

    void setupTypeControl(uint8_t controlType, BLDCMotor* motor){

        if(controlType == parameters::torqueVoltageMode){
            motor->torque_controller = TorqueControlType::voltage;
            motor->controller = MotionControlType::torque;

        }

        if(controlType == parameters::torqueVoltageMode){
            motor->torque_controller = TorqueControlType::dc_current;
            motor->controller = MotionControlType::velocity;

            motor->PID_current_q.P = parameters::P_torque;
            motor->PID_current_q.I = parameters::I_torque;
            motor->PID_current_q.D = parameters::D_torque;
            motor->LPF_current_q.Tf = parameters::Tf_torque;
            motor->PID_current_q.limit = motor->voltage_limit;
        }

        if(controlType == parameters::torqueVoltageMode){
            motor->torque_controller = TorqueControlType::dc_current;
            motor->controller = MotionControlType::velocity;

            motor->PID_current_q.P = parameters::P_torque;
            motor->PID_current_q.I = parameters::I_torque;
            motor->PID_current_q.D = parameters::D_torque;
            motor->LPF_current_q.Tf = parameters::Tf_torque;
            motor->PID_current_q.limit = motor->voltage_limit;

            motor->PID_velocity.P = parameters::P_vel;
            motor->PID_velocity.I = parameters::I_vel;
            motor->PID_velocity.D = parameters::D_vel;
            motor->LPF_velocity.Tf = parameters::Tf_vel;
            motor->PID_velocity.output_ramp = parameters::output_ramp_vel;
        }
    }

    void executeMotor(float targetMotor0, float targetMotor1){
        
        if(executeMotor0){
            motor_out0.loopFOC();
            motor_out0.move(targetMotor0);
        }

        if(executeMotor1){
            motor_out1.loopFOC();
            motor_out1.move(targetMotor1);    
        }
        
    }

}
