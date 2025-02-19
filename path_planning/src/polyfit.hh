#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <numeric>
#include <iostream>

Eigen::MatrixXd create_feature_matrix(Eigen::VectorXd& X,Eigen::VectorXd& Y, int degree=1)
{
    //Solves least squares problem A*C = Z for coefficients C
    int n = X.size();

    int num_terms = (degree + 1) * (degree + 2) / 2;
    Eigen::MatrixXd A(n, num_terms+1); //Vandermonde matrix

    for (int i=0;i<n;i++)
    {
        int col=0;
        for (int j=0;j <= degree;j++)
        {
            for (int k=0;k<j;k++)
            {
                A(i,col++) = std::pow(X[i],j-k) * std::pow(Y[i],k);
            }
        }
    }
    return A;
}

Eigen::VectorXd polyfit2D(std::vector<PathNode>& original_path, int degree) {
    int n = original_path.size();
    Eigen::VectorXd X(n);
    Eigen::VectorXd Y(n);
    Eigen::VectorXd z(n);
    for (int p=0;p<n;++p)
    {
        X[p] = original_path[p].pose[0];
        Y[p] = original_path[p].pose[1];
        z[p] = original_path[p].pose[2];
    }
    Eigen::MatrixXd A = create_feature_matrix(X,Y, degree);
    Eigen::VectorXd Z = Eigen::Map<const Eigen::VectorXd>(z.data(), z.size());

    // Solve for coefficients using least squares
    Eigen::VectorXd coeffs = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Z);
    return coeffs;
}

double evaluatePolynomial(const Eigen::VectorXd& coeffs, double x, double y, int order) {
    int col = 0;
    double result = 0.0;
    
    for (int j = 0; j <= order; ++j) {
        for (int k = 0; k <= j; ++k) {
            result += coeffs(col++) * std::pow(x, j - k) * std::pow(y, k);
        }
    }

    return result;
}

double computeFitQuality(const Eigen::VectorXd& coeffs, 
    std::vector<PathNode>& path,
    int order) 
    {

    int n = path.size();
    Eigen::VectorXd x(n);
    Eigen::VectorXd y(n);
    Eigen::VectorXd z(n);
    for (int p=0;p<n;++p)
    {
        x[p] = path[p].pose[0];
        y[p] = path[p].pose[1];
        z[p] = path[p].pose[2];
    }
    double sum_squared_residuals = 0.0;
    double sum_squared_total = 0.0;
    double mean_z = std::accumulate(z.begin(), z.end(), 0.0) / z.size();

    for (size_t i = 0; i < x.size(); ++i) {
        double predicted_z = 0.0;
        int col = 0;

        for (int j = 0; j <= order; ++j) 
        {
            for (int k = 0; k <= j; ++k) 
            {
                predicted_z += coeffs(col++) * std::pow(x[i], j - k) * std::pow(y[i], k);
                std::cout<<"predicted z"<<predicted_z<<std::endl;
            }
        }

    double residual = z[i] - predicted_z;
    sum_squared_residuals += residual * residual;
    sum_squared_total += (z[i] - mean_z) * (z[i] - mean_z);
    }

    double r_squared = (sum_squared_total > 1e-8) 
    ? 1.0 - (sum_squared_residuals / sum_squared_total)
    : 0.0;  // Default to zero if variance is too low


    return r_squared;
}