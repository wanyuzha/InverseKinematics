#include "IK.h"
#include "FK.h"
#include "minivectorTemplate.h"
#include <Eigen/Dense>
#include <adolc/adolc.h>
#include <cassert>
#if defined(_WIN32) || defined(WIN32)
  #ifndef _USE_MATH_DEFINES
    #define _USE_MATH_DEFINES
  #endif
#endif
#include <math.h>
using namespace std;

// CSCI 520 Computer Animation and Simulation
// Jernej Barbic and Yijing Li

namespace
{

// Converts degrees to radians.
template<typename real>
inline real deg2rad(real deg) { return deg * M_PI / 180.0; }

template<typename real>
Mat3<real> Euler2Rotation(const real angle[3], RotateOrder order)
{
  Mat3<real> RX = Mat3<real>::getElementRotationMatrix(0, deg2rad(angle[0]));
  Mat3<real> RY = Mat3<real>::getElementRotationMatrix(1, deg2rad(angle[1]));
  Mat3<real> RZ = Mat3<real>::getElementRotationMatrix(2, deg2rad(angle[2]));

  switch(order)
  {
    case RotateOrder::XYZ:
      return RZ * RY * RX;
    case RotateOrder::YZX:
      return RX * RZ * RY;
    case RotateOrder::ZXY:
      return RY * RX * RZ;
    case RotateOrder::XZY:
      return RY * RZ * RX;
    case RotateOrder::YXZ:
      return RZ * RX * RY;
    case RotateOrder::ZYX:
      return RX * RY * RZ;
  }
  assert(0);
}

// Performs forward kinematics, using the provided "fk" class.
// This is the function whose Jacobian matrix will be computed using adolc.
// numIKJoints and IKJointIDs specify which joints serve as handles for IK:
//   IKJointIDs is an array of integers of length "numIKJoints"
// Input: numIKJoints, IKJointIDs, fk, eulerAngles (of all joints)
// Output: handlePositions (world-coordinate positions of all the IK joints; length is 3 * numIKJoints)
template<typename real>
void forwardKinematicsFunction(
    int numIKJoints, const int * IKJointIDs, const FK & fk,
    const std::vector<real> & eulerAngles, std::vector<real> & handlePositions)
{
  // Students should implement this.
  // The implementation of this function is very similar to function computeLocalAndGlobalTransforms in the FK class.
  // The recommended approach is to first implement FK::computeLocalAndGlobalTransforms.
  // Then, implement the same algorithm into this function. To do so,
  // you can use fk.getJointUpdateOrder(), fk.getJointRestTranslation(), and fk.getJointRotateOrder() functions.
  // Also useful is the multiplyAffineTransform4ds function in minivectorTemplate.h .
  // It would be in principle possible to unify this "forwardKinematicsFunction" and FK::computeLocalAndGlobalTransforms(),
  // so that code is only written once. We considered this; but it is actually not easily doable.
  // If you find a good approach, feel free to document it in the README file, for extra credit.
    
    std::vector<Mat3<real>>    globalRotationMatrixAdouble;
    std::vector<Vec3<real>>    globalTranslationAdouble;
    globalRotationMatrixAdouble.resize(fk.getNumJoints());
    globalTranslationAdouble.resize(fk.getNumJoints());
  for (int traverse = 0; traverse < fk.getNumJoints(); traverse++)
  {
      int i = fk.getJointUpdateOrder(traverse);
      real orientEulerAngles[3];
      Mat3<real> orientRotationMatrix;
      Mat3<real> localRotationMatrix;
      // parent joint
      if (i == 0)
      {
          //get from orient and local euler angles fk class
          Vec3d tempD = fk.getJointOrient(i);
          orientEulerAngles[0] = tempD[0]; orientEulerAngles[1] = tempD[1]; orientEulerAngles[2] = tempD[2];

          orientRotationMatrix = Euler2Rotation(orientEulerAngles, RotateOrder::XYZ);
          real eulerA[3] = { eulerAngles[i*3] , eulerAngles[i*3+1] , eulerAngles[i*3+2]};
          localRotationMatrix = Euler2Rotation(eulerA, fk.getJointRotateOrder(i));

          globalRotationMatrixAdouble[i] = orientRotationMatrix * localRotationMatrix;
          tempD = fk.getJointRestTranslation(i);
          globalTranslationAdouble[i][0] = tempD[0]; globalTranslationAdouble[i][1] = tempD[1]; globalTranslationAdouble[i][2] = tempD[2];
      }
      else
      {
          //get from orient and local euler angles fk class
          Vec3d tempD = fk.getJointOrient(i);
          orientEulerAngles[0] = tempD[0]; orientEulerAngles[1] = tempD[1]; orientEulerAngles[2] = tempD[2];

          orientRotationMatrix = Euler2Rotation(orientEulerAngles, RotateOrder::XYZ);
          real eulerA[3] = { eulerAngles[i*3] , eulerAngles[i*3 + 1] , eulerAngles[i*3 + 2] };
          localRotationMatrix = Euler2Rotation(eulerA, fk.getJointRotateOrder(i));

          Vec3<real> localTranslationAdouble;
          tempD = fk.getJointRestTranslation(i);
          localTranslationAdouble[0] = tempD[0]; localTranslationAdouble[1] = tempD[1]; localTranslationAdouble[2] = tempD[2];

          int jointParent = fk.getJointParent(i);
          multiplyAffineTransform4ds(globalRotationMatrixAdouble[jointParent], globalTranslationAdouble[jointParent], orientRotationMatrix * localRotationMatrix,
              localTranslationAdouble, globalRotationMatrixAdouble[i], globalTranslationAdouble[i]);
      }
  }

    // getting handle positions from fk globalMatrix
    for (int i = 0; i < numIKJoints; i++)
    {
        int IKID = IKJointIDs[i];
        handlePositions[i*3] = globalTranslationAdouble[IKID][0];
        handlePositions[i*3+1] = globalTranslationAdouble[IKID][1];
        handlePositions[i*3+2] = globalTranslationAdouble[IKID][2];
    }
}

} // end anonymous namespaces

IK::IK(int numIKJoints, const int * IKJointIDs, FK * inputFK, int adolc_tagID)
{
  this->numIKJoints = numIKJoints;
  this->IKJointIDs = IKJointIDs;
  this->fk = inputFK;
  this->adolc_tagID = adolc_tagID;

  FKInputDim = fk->getNumJoints() * 3;
  FKOutputDim = numIKJoints * 3;

  // setting step
  input = new double[FKInputDim];
  output = new double[FKOutputDim];
  jacobian = new double[FKOutputDim * FKInputDim];
  jRow = new double* [FKOutputDim];
  for (int i = 0; i < FKOutputDim; i++)
  {
      jRow[i] = &jacobian[FKInputDim * i];
  }

  train_adolc();
}

void IK::train_adolc()
{
  // Students should implement this.
  // Here, you should setup adol_c:
  //   Define adol_c inputs and outputs. 
  //   Use the "forwardKinematicsFunction" as the function that will be computed by adol_c.
  //   This will later make it possible for you to compute the gradient of this function in IK::doIK
  //   (in other words, compute the "Jacobian matrix" J).
  // See ADOLCExample.cpp .
    trace_on(adolc_tagID);
    vector<adouble> eulerAngles(FKInputDim);
    for (int i = 0; i < FKInputDim; i++)
    {
        eulerAngles[i] <<= 0.0;
    }
    vector<adouble> handlePositions(FKOutputDim);
    forwardKinematicsFunction(numIKJoints, IKJointIDs, *fk, eulerAngles, handlePositions);
    vector<double> output(FKOutputDim);
    for (int i = 0; i < FKOutputDim; i++)
    {
        handlePositions[i] >>= output[i];
    }
    trace_off();

}

void IK::train_adolc_test()
{
    // Students should implement this.
    // Here, you should setup adol_c:
    //   Define adol_c inputs and outputs. 
    //   Use the "forwardKinematicsFunction" as the function that will be computed by adol_c.
    //   This will later make it possible for you to compute the gradient of this function in IK::doIK
    //   (in other words, compute the "Jacobian matrix" J).
    // See ADOLCExample.cpp .
    int n = 3; // input dimension is n
    int m = 2; // output dimension is m
    trace_on(adolc_tagID);
    vector<adouble> x(n); // define the input of the function f
    for (int i = 0; i < n; i++)
        x[i] <<= 0.0; // The <<= syntax tells ADOL-C that these are the input variables.

    vector<adouble> y(m); // define the output of the function f

    // The computation of f goes here:
    y[0] = x[0] + 2 * x[1] + 3 * x[2];
    y[1] = 4 * x[0] + 5 * x[1];

    vector<double> output(m);
    for (int i = 0; i < m; i++)
        y[i] >>= output[i]; // Use >>= to tell ADOL-C that y[i] are the output variables

    trace_off();

}

void IK::doIK(const Vec3d* targetHandlePositions, Vec3d* jointEulerAngles)
{
    // You may find the following helpful:
    int numJoints = fk->getNumJoints(); // Note that is NOT the same as numIKJoints!

    // Students should implement this.
    // Use adolc to evalute the forwardKinematicsFunction and its gradient (Jacobian). It was trained in train_adolc().
    // Specifically, use ::function, and ::jacobian .
    // See ADOLCExample.cpp .
    //
    // Use it implement the Tikhonov IK method (or the pseudoinverse method for extra credit).
    // Note that at entry, "jointEulerAngles" contains the input Euler angles. 
    // Upon exit, jointEulerAngles should contain the new Euler angles.
    
    

    int N = 5;
    for (int step = 0; step < N; step++)
    {
        for (int i = 0; i < numJoints; i++)
        {
            input[i * 3] = jointEulerAngles[i][0];
            input[i * 3 + 1] = jointEulerAngles[i][1];
            input[i * 3 + 2] = jointEulerAngles[i][2];
        }

        ::function(adolc_tagID, FKOutputDim, FKInputDim, input, output);
        Eigen::VectorXd b(FKOutputDim);
        for (int i = 0; i < numIKJoints; i++)
        {
            b(i * 3) = (targetHandlePositions[i][0] - output[i * 3])/N;
            b(i * 3 + 1) = (targetHandlePositions[i][1] - output[i * 3 + 1])/N;
            b(i * 3 + 2) = (targetHandlePositions[i][2] - output[i * 3 + 2])/N;
        }

        ::jacobian(adolc_tagID, FKOutputDim, FKInputDim, input, jRow);

        Eigen::MatrixXd J(FKOutputDim, FKInputDim);
        for (int i = 0; i < FKOutputDim; i++)
        {
            for (int j = 0; j < FKInputDim; j++)
            {
                J(i, j) = jacobian[i * FKInputDim + j];
            }
        }
        Eigen::MatrixXd A = J.transpose() * J + 0.001 * Eigen::MatrixXd::Identity(FKInputDim, FKInputDim);
        Eigen::VectorXd x = A.ldlt().solve(J.transpose() * b);
        //Eigen::VectorXd x = J.transpose() * (J * J.transpose()).inverse() * b;
        for (int i = 0; i < numJoints; i++)
        {
            jointEulerAngles[i][0] += x(i * 3);
            jointEulerAngles[i][1] += x(i * 3 + 1);
            jointEulerAngles[i][2] += x(i * 3 + 2);
        }
    }

}

