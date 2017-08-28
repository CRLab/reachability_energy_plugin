#include "reachabilityEnergy.h"
#include "graspit/robot.h"
#include "graspit/grasp.h"
#include "graspit/debug.h"
#include "graspit/world.h"
#include "graspit/contact/virtualContact.h"
#include "graspit/quality/quality.h"


#include <fstream>
// #define DEBUG

#include <QSettings>


ReachabilityEnergy::ReachabilityEnergy()
{
//    export REACHABILITY_CONFIG_FILE=/home/ireti/ros/reachability_paper_experiments/src/graspit-ros/graspit/graspit_source/reachability.conf
    QString configDir = getenv("REACHABILITY_CONFIG_DIR");
    QString configFile = configDir + QString("/reachability.conf");
#ifdef DEBUG
    std::cout << "configDir: \t" << configDir.toStdString() << std::endl;
    std::cout << "configFile: \t" << configFile.toStdString() << std::endl;
#endif

    QSettings settings(configFile,  QSettings::NativeFormat);
    string filename = configDir.toStdString() + "/processed/reach_data";
    string filename_objBaseTrans = configDir.toStdString() + "/object_pose_in_reference_frame.csv";

    contact_coeff = settings.value("reachability_config/contact_energy_coeff").toDouble();
    potential_coeff = settings.value("reachability_config/potential_energy_coeff").toDouble();
    reachability_coeff = settings.value("reachability_config/reachability_energy_coeff").toDouble();
    

#ifdef DEBUG
    std::cout << "reachability data filename: " << filename << std::endl;
    std::cout << "filename_objBaseTrans: " << filename_objBaseTrans << std::endl;
    std::cout << "contact_coeff: " << contact_coeff << std::endl;
    std::cout << "potential_coeff: " << potential_coeff << std::endl;
    std::cout << "reachability_coeff: " << reachability_coeff << std::endl;
#endif


    // load reachability data
    load_file_templated (filename+".dims", dims);
    load_file_templated (filename+".mins", mins);
    load_file_templated (filename+".step", steps);

    load_ND_Array (filename+".full", reachSpaceArrayFull);
    load_ND_Array (filename+".sdf", reachSpaceArraySDF);

    if (dims.size() == 3)
    {
      reachSpaceTensorFull = Eigen::TensorMap<Eigen::Tensor<double, 3, Eigen::RowMajor>> (reachSpaceArrayFull, dims[0], dims[1], dims[2]);
      reachSpaceTensorSDF = Eigen::TensorMap<Eigen::Tensor<double, 3, Eigen::RowMajor>> (reachSpaceArraySDF, dims[0], dims[1], dims[2]);
      // cout << "reachSpaceTensorSDF.minimum(): \t" << reachSpaceTensorSDF.minimum() << endl;
      // cout << "reachSpaceTensorSDF.maximum(): \t" << reachSpaceTensorSDF.maximum() << endl;
      Eigen::Tensor<double, 0, Eigen::RowMajor> tempMin = reachSpaceTensorSDF.minimum();
      reach_min = *tempMin.data();
      Eigen::Tensor<double, 0, Eigen::RowMajor> tempMax = reachSpaceTensorSDF.maximum();
      reach_max = *tempMax.data();

      #ifdef DEBUG
          cout << "reachSpaceTensorSDF.minimum(): \t" << reach_min << endl;
          cout << "reachSpaceTensorSDF.maximum(): \t" << reach_max << endl;
          cout << "reachSpaceTensorSDF(2,1,0): \t" << reachSpaceTensorSDF(2,1,0) << endl;
      #endif
    }

    if (dims.size() == 6)
    {
      reachSpaceTensorFull_6D = Eigen::TensorMap<Eigen::Tensor<double, 6, Eigen::RowMajor>> (reachSpaceArrayFull, dims[0], dims[1], dims[2], dims[3], dims[4], dims[5]);
      reachSpaceTensorSDF_6D = Eigen::TensorMap<Eigen::Tensor<double, 6, Eigen::RowMajor>> (reachSpaceArraySDF, dims[0], dims[1], dims[2], dims[3], dims[4], dims[5]);
      Eigen::Tensor<double, 0, Eigen::RowMajor> tempMin = reachSpaceTensorSDF_6D.minimum();
      reach_min = *tempMin.data();
      Eigen::Tensor<double, 0, Eigen::RowMajor> tempMax = reachSpaceTensorSDF_6D.maximum();
      reach_max = *tempMax.data();

      #ifdef DEBUG
          cout << "reachSpaceTensorSDF_6D.minimum(): \t" << reach_min << endl;
          cout << "reachSpaceTensorSDF_6D.maximum(): \t" << reach_max << endl;
          cout << "reachSpaceTensorSDF_6D(2,1,0,0,3,2): \t" << reachSpaceTensorSDF_6D(2,1,0,0,3,2) << endl;
      #endif
    }

    // load object-base transform
    load_file_as_eigen_matrix (filename_objBaseTrans, objectBaseTrans);
#ifdef DEBUG
    std::cout << "object-base transform: \t "<< std::endl << objectBaseTrans << std::endl;
#endif
}


double ReachabilityEnergy::energy() const
{
  double reachableEnergy = reachableQualityEnergy();

  return -reachableEnergy;
}



double ReachabilityEnergy::reachableQualityEnergy() const
{
    // get hand transform in object frame
    transf transHO = mObject->getTran().inverse() % mHand->getTran();
    Eigen::MatrixXd queryPoseInObjectFrame(4,4);
    queryPoseInObjectFrame.block(0, 0, 3, 3) = transHO.affine();
    queryPoseInObjectFrame.block(0, 3, 3, 1) = transHO.translation();
    queryPoseInObjectFrame(3, 0) = 0;
    queryPoseInObjectFrame(3, 1) = 0;
    queryPoseInObjectFrame(3, 2) = 0;
    queryPoseInObjectFrame(3, 3) = 1;


    queryPoseInObjectFrame(0, 3) /= 1000;
    queryPoseInObjectFrame(1, 3) /= 1000;
    queryPoseInObjectFrame(2, 3) /= 1000;
    Eigen::MatrixXd queryPoseInBaseLinkFrame = objectBaseTrans * queryPoseInObjectFrame;


    // get xyzRPY as eigen vector
    Eigen::VectorXd queryVecInBaseLinkFrame =  trans2xyzRPY (queryPoseInBaseLinkFrame);
//    cout << "queryPoseInBaseLinkFrame : \t" << queryPoseInBaseLinkFrame << endl;
//    cout << "queryVecInBaseLinkFrame : \t" << queryVecInBaseLinkFrame << endl;
    return interpolateReachability( queryVecInBaseLinkFrame);
}



// double ReachabilityEnergy::interpolateReachability(
//        Eigen::VectorXd query
//        ) const
// {

//     #ifdef DEBUG
//    cout << "USING APPROXIMATE ReachabilityEnergy::interpolateReachability" << endl;
//    cout << "query: \n" << endl << query << endl;
//     #endif
//    int ndims = dims.size();

//    // // query roll and yaw calculated using atan2 is between -pi and pi
//    // // however, data generation was between 0 and 2pi
//    // if  (query[3] < 0)
//    //     query[3] += 2*M_PI;            // add 2pi to normalize range to 0-2pi
//    // if  (query[5] < 0)
//    //     query[5] += 2*M_PI;            // add 2pi to normalize range to 0-2pi

//    Eigen::VectorXi indicesFloor(ndims);
//    Eigen::VectorXi indicesCeiling(ndims);
//    Eigen::VectorXd floorDistance(ndims);
//    Eigen::VectorXd ceilDistance(ndims);
//    for (int i = 0; i < ndims; ++i)
//    {
//        indicesFloor[i] = (query[i] - mins[i]) / steps[i];
//        indicesCeiling[i] = (query[i] - mins[i]) / steps[i] + 1;
//        floorDistance[i] = (query[i] - (mins[i] +indicesFloor[i]*steps[i]) );
//        ceilDistance[i] = steps[i] - floorDistance[i];

//        if(indicesFloor[i] < 0){
//            cout << "INDEX OUT OF DIMENSION i: "<< i <<" indices[i]: \t" << indicesFloor[i]  << endl;
//            cout << "query[i]: "<< query[i] <<" mins[i]: \t" << mins[i] <<" steps[i]: \t" << steps[i]   << endl;
//            indicesFloor[i] = 0;
//            indicesCeiling[i] = 0;
//        }
//        if(indicesCeiling[i] > dims[i]){
//            cout << "INDEX OUT OF DIMENSION i: "<< i <<" indices[i]: \t" << indicesFloor[i]  << endl;
//            indicesFloor[i] = dims[i];
//            indicesCeiling[i] = dims[i];
//        }
//    }

//     #ifdef DEBUG
//    cout << "indicesFloor: \n" << endl << indicesFloor << endl;
//     #endif

//    double val1;
//    double val2;
//   if (dims.size() == 3)
//     {
//       val1 = reachSpaceTensorSDF(indicesFloor[0],indicesFloor[1],indicesFloor[2]);
//       val2 = reachSpaceTensorSDF(indicesCeiling[0],indicesCeiling[1],indicesCeiling[2]);
//     }
//   if (dims.size() == 6)
//     {
//       val1 = reachSpaceTensorSDF_6D(indicesFloor[0],indicesFloor[1],indicesFloor[2],indicesFloor[3],indicesFloor[4],indicesFloor[5]);
//       val2 = reachSpaceTensorSDF_6D(indicesCeiling[0],indicesCeiling[1],indicesCeiling[2],indicesCeiling[3],indicesCeiling[4],indicesCeiling[5]);
//     }

//    double val = (ceilDistance.norm() * val1 + floorDistance.norm() * val2) /(floorDistance.norm() + ceilDistance.norm() );

//     #ifdef DEBUG
//    cout << "mHand->getTran(): \t" << mHand->getTran() << endl;
//    cout << "floorDistance.norm(): \t" << floorDistance.norm() << endl;
//    cout << "ceilDistance.norm(): \t" << ceilDistance.norm() << endl;
//    cout << "val1: \t" << val1 << endl;
//    cout << "val2: \t" << val2 << endl;
//    cout << "val: \t" << val << endl;
//    #endif


//    return val;

// }





double ReachabilityEnergy::interpolateReachability(
     Eigen::VectorXd query
     ) const
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
        // int full_index = ravel_multi_index(index, dims);
        // cornerValues[i] = ((double *)reachSpaceArraySDF)[full_index];

        if (dims.size() == 3)
        {
         cornerValues[i] = reachSpaceTensorSDF(index[0],index[1],index[2]);
          #ifdef DEBUG
            cout << "cornerValues[i]: \t" << cornerValues[i] << endl;
            cout << "reachSpaceTensorSDF(index[0],index[1],index[2]):\t" << reachSpaceTensorSDF(index[0],index[1],index[2]) << endl;
          #endif
        }

        if (dims.size() == 6)
        {
          cornerValues[i] = reachSpaceTensorSDF_6D(index[0],index[1],index[2],index[3],index[4],index[5]);
          #ifdef DEBUG
            cout << "cornerValues[i]: \t" << cornerValues[i] << endl;
            cout << "reachSpaceTensorSDF_6D(index[0],index[1],index[2],index[3],index[4],index[5]):\t" << reachSpaceTensorSDF_6D(index[0],index[1],index[2],index[3],index[4],index[5]) << endl;
          #endif
        }

        interpWeights[i] = weight;
     }
     double num = (cornerValues.array() * interpWeights.array()).sum() ;
     double denom = interpWeights.array().sum();
     double result = num / denom;
   return result;
}



double ReachabilityEnergy::contactEnergy() const
{
    mHand->getGrasp()->collectVirtualContacts();

    //DBGP("Contact energy computation")
    //average error per contact
    VirtualContact *contact;
    vec3 p, n, cn;
    double totalError = 0;
    for (int i = 0; i < mHand->getGrasp()->getNumContacts(); i++)
    {
        contact = (VirtualContact *)mHand->getGrasp()->getContact(i);
        contact->getObjectDistanceAndNormal(mObject, &p, NULL);
        double dist = p.norm();

        //this should never happen anymore since we're never inside the object
        //if ( (-1.0 * p) % n < 0) dist = -dist;

        //BEST WORKING VERSION, strangely enough
        totalError += fabs(dist);

        //let's try this some more
        //totalError += distanceFunction(dist);
        //cn = -1.0 * contact->getWorldNormal();

        //new version
        cn = contact->getWorldNormal();
        n = p.normalized();
        double d = 1 - cn.dot(n);
        totalError += d * 100.0 / 2.0;
    }

    totalError /= mHand->getGrasp()->getNumContacts();

    //DBGP("Contact energy: " << totalError);
    return totalError;
}

double ReachabilityEnergy::potentialQualityEnergy() const
{
    bool verbose = false;
    VirtualContact *contact;
    vec3 p, n, cn;
    int count = 0;
    //DBGP("Potential quality energy computation")
    for (int i = 0; i < mHand->getGrasp()->getNumContacts(); i++)
    {
        contact = (VirtualContact *)mHand->getGrasp()->getContact(i);
        contact->computeWrenches(true, false);
        contact->getObjectDistanceAndNormal(mObject, &p, NULL);
        n = contact->getWorldNormal();
        double dist = p.norm();
        p = p.normalized();
        double cosTheta = n.dot(p);
        double factor = potentialQualityScalingFunction(dist, cosTheta);
        if (verbose)
        {
            fprintf(stderr, "VC %d on finger %d link %d\n", i, contact->getFingerNum(), contact->getLinkNum());
            fprintf(stderr, "Distance %f cosTheta %f\n", dist, cosTheta);
            fprintf(stderr, "Scaling factor %f\n\n", factor);
        }
        contact->scaleWrenches(factor);
        if (factor > 0.25)
        {
            count++;
            // contact->mark(true);
        } else {
          // contact->mark(false);
        }
    }
    double gq = -1;
    //to make computations more efficient, we only use a 3D approximation
    //of the 6D wrench space
    std::vector<int> forceDimensions(6, 0);
    forceDimensions[0] = forceDimensions[1] = forceDimensions[2] = 1;
    if (count >= 3) {
        mHand->getGrasp()->updateWrenchSpaces(forceDimensions);
        gq = mEpsQual->evaluate();
    }
    if (verbose) {
        fprintf(stderr, "Quality: %f\n\n", gq);
    }
    if (count) {
        DBGP("Count: " << count << "; Gq: " << gq << ";");
    }
    return -gq;
    return 10;
}

double
ReachabilityEnergy::potentialQualityScalingFunction(double dist, double cosTheta) const
{
    double sf = 0;
    /*
  (void*)&cosTheta;
  if (dist<0) return 0;
  sf += 100.0 / ( pow(2.0,dist/15.0) );
  //comment this out if you don't care about normals
  //sf += 100 - (1 - cosTheta) * 100.0 / 2.0;
  */
    /*
  if (cosTheta < 0.8) return 0; //about 35 degrees
  sf = 10.0 / ( pow((double)3.0,dist/25.0) );
  if (sf < 0.25) sf = 0; //cut down on computation for tiny values. more than 50mm away
  return sf;
  */
    /*
  if (cosTheta < 0.8) return 0; //about 35 degrees
  if (dist < 20) sf = 1;
  else {
  sf = 1.5 - 1.5 / ( pow((double)3.0,(dist-20)/25.0) );
  sf = cos( sf*sf ) + 1;
  }
  sf = sf*10;
  if (sf < 0.25) sf = 0; //cut down on computation for tiny values. more than 50mm away
  return sf;
  */
    if (cosTheta < 0.7) { return 0; }
    if (dist > 50) { return 0; }
    sf = cos(3.14 * dist / 50.0) + 1;
    return sf;
}

