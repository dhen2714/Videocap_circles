#include <VideoCap.hpp>

GaussianFit::GaussianFit(std::vector<std::pair<double, double>>& data, 
    double mu, double sigma, double amplitude, double base)
{
    data_samples = data;
    params = mu,
             sigma,
             amplitude,
             base;
}

double GaussianFit::model(const double& x, const parameter_vector& params)
{
    const double mu = params(0);
    const double sigma = params(1);
    const double amplitude = params(2);
    const double base = params(3);

    const double temp = ((x-mu)/sigma);

    return (amplitude * exp(-0.5*temp*temp) + base);
}

double GaussianFit::residual(const std::pair<double, double>& data,
    const parameter_vector& params)
{
    return model(data.first, params) - data.second;
}

parameter_vector GaussianFit::residual_derivative(const std::pair<double, double>& data,
    const parameter_vector& params)
{
    parameter_vector der;

    const double mu = params(0);
    const double sigma = params(1);
    const double amplitude = params(2);

    const double x = data.first;

    const double temp = ((x-mu)/sigma);

    der(0) = ((x-mu)/(sigma*sigma)) * amplitude * exp(-0.5*temp*temp);
    der(1) = (((x-mu)*(x-mu))/(2*sigma)) * amplitude * exp(-0.5*temp*temp);
    der(2) = exp(-0.5*temp*temp);
    der(3) = 1;

    return der;
}

void GaussianFit::fit()
{
    solve_least_squares_lm(dlib::objective_delta_stop_strategy(1e-7), 
                        residual,
                        residual_derivative,
                        data_samples,
                        params);
}
