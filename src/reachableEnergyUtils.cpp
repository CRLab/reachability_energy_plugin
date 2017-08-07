#include "graspit/transform.h"

#include "reachableEnergyUtils.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include "vector"

#include <ctime>
#include <cstdlib>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using namespace std;

void load_file_as_eigen_matrix (const std::string filename, Eigen::MatrixXd &D) {
    std::vector<std::vector<double> > vD;
    std::string line;
    std::fstream in;
    in.open(filename.c_str());
    while (in)
    {
        std::getline(in, line);
        std::vector<double> row;
        std::stringstream stream_line(line);
        double value;
        while (stream_line >> value) row.push_back(value);
        if (!row.empty()) vD.push_back(row);

    }
        // cout << "checking size vD "<< vD.size()  << endl;

    std_vector_2_eigen_matrix ( vD, D);
}

void std_vector_2_eigen_matrix (const std::vector<std::vector<double> > vD, Eigen::MatrixXd &D) {
    try {

        int n_rows = vD.size();
        int n_cols = vD[0].size();

        // cout << "n_rows: " << n_rows <<", n_cols: " << n_cols<< endl;

        D.resize(n_rows, n_cols);

        for (int i = 0; i < n_rows; ++i)
        {
            for (int j = 0; j < n_cols; ++j)
            {
                D(i, j) = vD[i][j];

                //cout << "checking size "<< row.size()  << endl;
                if((n_cols== 8) && (j==7)) {
        //            cout << "checking size " << endl;
                    if((vD[i][j] < 0) || (vD[i][j] > 1)) {
                        cout << "target Value greater than 1: \t"<< vD[i][j] << ", line number:\t" << i << endl;
                    }
                }
            }
        }
    } catch (const std::exception& e) {

        cout << "Error! std_vector_2_eigen_matrix!!" << endl;
    }
    // cout << D.rows() << "," << D.cols() << endl;
    // cout << D << endl;
}

void xyzrpy2Trans (double x, double y, double z, double roll, double pitch, double yaw, Eigen::MatrixXd &trans) {
//    cout << "roll: \t" <<roll << endl;
//    cout << "pitch: \t" <<pitch << endl;
//    cout << "yaw: \t" <<yaw << endl;
//    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
//    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
//    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

//    Eigen::Quaternion<double> q = rollAngle * pitchAngle * yawAngle;
//    Eigen::Matrix3d rotationMatrix = q.matrix();

    Eigen::MatrixXd rotationMatrix  = transf::RPY(roll, pitch, yaw).affine();

    trans.resize(4, 4);
    trans.block(0, 0, 3, 3) = rotationMatrix;
    trans(0, 3) = x;
    trans(1, 3) = y;
    trans(2, 3) = z;
    trans(3, 0) = 0.0;
    trans(3, 1) = 0.0;
    trans(3, 2) = 0.0;
    trans(3, 3) = 1;

//    cout << "transf::RPY(roll, pitch, yaw).affine(): \t" << endl << rotationMatrix << endl;

    Eigen::VectorXd xyzRPY = trans2xyzRPY(trans);
//    cout << "trans2xyzRPY(rotationMatrix): \t" << endl << xyzRPY << endl << endl;
}

Eigen::MatrixXd invertTrans (Eigen::MatrixXd trans) {
    return trans.inverse();
}

Eigen::MatrixXd composeTrans (Eigen::MatrixXd &trans1, Eigen::MatrixXd &trans2) {
    return trans1 * trans2;
}

Eigen::VectorXd trans2xyzRPY (Eigen::MatrixXd &trans) {

    Eigen::VectorXd xyzRPY(6);
    xyzRPY(0) = trans(0, 3);
    xyzRPY(1) = trans(1, 3);
    xyzRPY(2) = trans(2, 3);

    double theta = atan2(-trans(2,0), sqrt(trans(0,0)*trans(0,0) + trans(1,0)*trans(1,0)));
    xyzRPY(3) = atan2(trans(2,1)/cos(theta), trans(2,2)/cos(theta));
    xyzRPY(4) = theta;
    xyzRPY(5) = atan2(trans(1,0)/cos(theta), trans(0,0)/cos(theta));

    return xyzRPY;
}



Eigen::MatrixXd normalize_data(const Eigen::MatrixXd M, Eigen::MatrixXd stepSize)
{
    Eigen::MatrixXd M_normalized(M.rows(), M.cols());
    for (int i = 0; i < M.cols(); ++i)
    {
        M_normalized.col(i) = M.col(i) / stepSize(i);
    }
    return M_normalized;
}




/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////


void load_data_6dim(const std::string filename, double* dataArray) {

    std::ifstream in(filename.c_str(), std::ifstream::ate | std::ifstream::binary);

    int fileSize = in.tellg();
    int tokensCount = fileSize / sizeof(double);

    std::ifstream in2(filename.c_str(), std::ifstream::binary);
    in2.read((char*)dataArray, tokensCount * sizeof(double));

}



void load_ND_Array (const std::string filename, double* &array) {

  // get file size
  std::ifstream in(filename.c_str(), std::ifstream::ate | std::ifstream::binary);
  int fileSize = in.tellg();
  int tokensCount = fileSize / sizeof(double);

  array = new double[tokensCount];

  // read data into array
  std::ifstream in2(filename.c_str(), std::ifstream::binary);
  in2.read((char*)array, tokensCount * sizeof(double));

}


int ravel_multi_index(
  const Eigen::VectorXi index,
  const Eigen::VectorXi dims
  ){
  // get direct index
   // similar to np.ravel_multi_index((3,1,4,1), (6,7,8,9))
  int ndims = dims.size();
  int full_index = index[0];
  for (int dim = 1; dim <= ndims - 1; ++dim)
  {
      full_index = (full_index * dims[dim]) + index[dim];
  }
  return full_index;
}



double interpolateQuery(
     double* &array,
     Eigen::VectorXi dims,
     Eigen::VectorXd mins,
     Eigen::VectorXd steps,
     Eigen::VectorXd query
     )
{
  // Eigen::Tensor<double, 6, Eigen::RowMajor> dataTensor(dims[0], dims[1], dims[2], dims[3], dims[4], dims[5]);
  // dataTensor = Eigen::TensorMap<Eigen::Tensor<double, 6, Eigen::RowMajor>> (array, dims[0], dims[1], dims[2], dims[3], dims[4], dims[5]);

  // int DIM = 3;
  // Eigen::Tensor<double, 3, Eigen::RowMajor> dataTensor(dims[0], dims[1], dims[2]);
  // dataTensor = Eigen::TensorMap<Eigen::Tensor<double, 3, Eigen::RowMajor>> (array, dims[0], dims[1], dims[2]);

  int ndims = dims.size();
  int count = 1 << ndims;      // 000000
  Eigen::VectorXd ndivs(ndims);

    for (int i = 0; i < ndims; ++i)
   {
       ndivs[i] = (query[i] - mins[i]) / steps[i];

       if(ndivs[i] < 0){
           ndivs[i] = 0;
       }
       if(ndivs[i] >= dims[i]){
           ndivs[i] = dims[i]-1;
       }
   }

   // 000000
   Eigen::VectorXd cornerValues(count);
   Eigen::VectorXd interpWeights(count);

   for (int i = 0; i < count; ++i)
   {
       Eigen::VectorXd weights(ndims);
       Eigen::VectorXi index(ndims);
       double weight = 1;
       // for each dimension get the weight
       for (int pos = 0; pos < ndims; ++pos)
       {
           int after = (i >> pos) & 0x1;        // either zero or one
           double alpha = ndivs[pos] - int(ndivs[pos]);   // fraction between 0 and 1

           weights[pos] = ((1.0 - after) * (1.0 - alpha) + (after) * (alpha));

           weight *= weights[pos];
           index[pos] = int(ndivs[pos]) + after;
       }

      // collate index and weight
        int full_index = ravel_multi_index(index, dims);
        cornerValues[i] = array[full_index];
        interpWeights[i] = weight;
     }
     double num = (cornerValues.array() * interpWeights.array()).sum() ;
     double denom = interpWeights.array().sum();
     double result = num / denom;
   return result;
}
