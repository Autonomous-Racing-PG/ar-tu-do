#include "pid_controller.h"

PIDController::PIDController(float p, float i, float d)
    : m_p{ p }
    , m_i{ i }
    , m_d{ d }
    , m_previous_error{ 0 }
    , m_integral{ 0 }
{
}

float PIDController::updateAndGetCorrection(float error, float deltaTime)
{
    this->m_integral += error * deltaTime;
    float derivative = (error - this->m_previous_error) / deltaTime;
    this->m_previous_error = error;

    return this->m_p * error + this->m_i * this->m_integral + this->m_d * derivative;
}