#include <jaco_regressor/jacoRegressorClass.h>

jacoRegressorClass::jacoRegressorClass()
{
    gravity_param.setOnes();
    //printParam();
    jaco_regressor_initialize();
}

jacoRegressorClass::~jacoRegressorClass()
{
    jaco_regressor_terminate();
}


void jacoRegressorClass::genRegressorMatrix(double q2, double q3, double q4,
        double q5, double q6, JacoRegressorMatrix &jacoMatrix)
{
    double regressor_array[60];
    jaco_regressor(q2, q3, q4, q5, q6, regressor_array);
    std::vector<double> regressorVector;
    for (unsigned int i = 0; i < 60; i++)
    {
        regressorVector.push_back(regressor_array[i]);
    }

    for (unsigned int row = 0; row < 5; row++)
    {
        for (unsigned int col = 0; col < 12; col++)
        {
            jacoMatrix(row, col) = regressorVector[col * 5 + row];
        }
    }

}

void jacoRegressorClass::genJointTorque(std::vector<double> posi_vector, JointVector &torque_vector)
{
    JacoRegressorMatrix regressor_matrix;
    genRegressorMatrix(posi_vector[1], posi_vector[2], posi_vector[3],
                       posi_vector[4], posi_vector[5], regressor_matrix);
    torque_vector << 0, regressor_matrix * gravity_param; //5*1 vector, add 0 in the first place
}

void jacoRegressorClass::printParam()
{
    std::cout << "Parameter vector of gravity: " << std::endl;
    for (unsigned int i = 0; i < 12; i++)
    {
        std::cout << gravity_param(i) << " | ";
    }
    std::cout << std::endl;
}

void jacoRegressorClass::setParam(JacoParamVector p)
{
    gravity_param = p;
    //printParam();
}

JacoParamVector jacoRegressorClass::getParam() const
{
    return gravity_param;
}
