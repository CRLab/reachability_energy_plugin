

#ifndef _reachableenergyutils_h_
#define _reachableenergyutils_h_

#include <iostream>
#include <fstream>
#include <sstream>
#include "vector"

#include <ctime>
#include <cstdlib>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using namespace std;

void load_file_as_eigen_matrix (const std::string filename, Eigen::MatrixXd &D) ;
void std_vector_2_eigen_matrix (const std::vector<std::vector<double> > vD, Eigen::MatrixXd &D);
void xyzrpy2Trans (double x, double y, double z, double roll, double pitch, double yaw, Eigen::MatrixXd &trans);
Eigen::MatrixXd invertTrans (Eigen::MatrixXd trans);
Eigen::MatrixXd composeTrans (Eigen::MatrixXd &trans1, Eigen::MatrixXd &trans2);
Eigen::VectorXd trans2xyzRPY (Eigen::MatrixXd &trans);

Eigen::MatrixXd normalize_data(const Eigen::MatrixXd M, Eigen::MatrixXd stepSize);

double interpolation_nearest_neighbors(Eigen::VectorXd targetData, Eigen::VectorXd indices, Eigen::VectorXd dists);


void interpolation(
    std::vector<double> query,
    std::vector<double> stepSize,
    std::vector<double> dims,
    std::vector<double> mins) ;


void interpolationEigen(Eigen::VectorXd query,
    Eigen::VectorXd stepSize,
    Eigen::VectorXd dims,
    Eigen::VectorXd mins, Eigen::VectorXd &indices, Eigen::VectorXd &dists);



/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////



template<typename T>
void load_file_templated (const std::string filename, std::vector<T>& row) {
    std::fstream in;
    in.open(filename.c_str());
    std::string line;

        std::getline(in, line);
        std::stringstream stream_line(line);

        T value;
        while (stream_line >> value)
        {
            row.push_back(value);
            if (stream_line.peek() == ',')
                stream_line.ignore();
        }
        in.close();
    cout << row.size() << endl;
    for (int i = 0; i < row.size(); ++i)
        cout << row[i] << ", ";
    cout << endl;
}

template<typename Scalar>
void load_file_templated (const std::string filename, Eigen::Matrix<Scalar, Eigen::Dynamic, 1> & row) {
    std::vector<Scalar> temp;
    load_file_templated (filename, temp);

    row.resize(temp.size());
    row = Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, 1> >(temp.data(), temp.size());

}

void load_data_6dim(const std::string filename, double* dataArray);


/////////////////////////////////////////////////////////////////////////////////

void load_ND_Array (const std::string filename, double* &array);

int ravel_multi_index(
  const Eigen::VectorXi index,
  const Eigen::VectorXi dims
  );

double interpolateQuery(
     double* &array,
     Eigen::VectorXi dims,
     Eigen::VectorXd mins,
     Eigen::VectorXd steps,
     Eigen::VectorXd query
     );

#endif
