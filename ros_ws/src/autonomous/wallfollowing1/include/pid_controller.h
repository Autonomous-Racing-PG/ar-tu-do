#pragma once

class PIDController
{
    public:
    PIDController(float p, float i, float d);

    float updateAndGetCorrection(float error, float deltaTime);

    private:
    float m_p;
    float m_i;
    float m_d;

    float m_previous_error;
    float m_integral;
};
