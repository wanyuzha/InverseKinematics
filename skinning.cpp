#include "skinning.h"
#include "vec3d.h"
#include <algorithm>
#include <cassert>
#include <iostream>
#include <fstream>
#include "dual_quaternion.h"

using namespace std;

// CSCI 520 Computer Animation and Simulation
// Jernej Barbic and Yijing Li

Skinning::Skinning(int numMeshVertices, const double * restMeshVertexPositions,
    const std::string & meshSkinningWeightsFilename)
{
  this->numMeshVertices = numMeshVertices;
  this->restMeshVertexPositions = restMeshVertexPositions;

  cout << "Loading skinning weights..." << endl;
  ifstream fin(meshSkinningWeightsFilename.c_str());
  assert(fin);
  int numWeightMatrixRows = 0, numWeightMatrixCols = 0;
  fin >> numWeightMatrixRows >> numWeightMatrixCols;
  assert(fin.fail() == false);
  assert(numWeightMatrixRows == numMeshVertices);
  int numJoints = numWeightMatrixCols;

  vector<vector<int>> weightMatrixColumnIndices(numWeightMatrixRows);
  vector<vector<double>> weightMatrixEntries(numWeightMatrixRows);
  fin >> ws;
  while(fin.eof() == false)
  {
    int rowID = 0, colID = 0;
    double w = 0.0;
    fin >> rowID >> colID >> w;
    weightMatrixColumnIndices[rowID].push_back(colID);
    weightMatrixEntries[rowID].push_back(w);
    assert(fin.fail() == false);
    fin >> ws;
  }
  fin.close();

  // Build skinning joints and weights.
  numJointsInfluencingEachVertex = 0;
  for (int i = 0; i < numMeshVertices; i++)
    numJointsInfluencingEachVertex = std::max(numJointsInfluencingEachVertex, (int)weightMatrixEntries[i].size());
  assert(numJointsInfluencingEachVertex >= 2);

  // Copy skinning weights from SparseMatrix into meshSkinningJoints and meshSkinningWeights.
  meshSkinningJoints.assign(numJointsInfluencingEachVertex * numMeshVertices, 0);
  meshSkinningWeights.assign(numJointsInfluencingEachVertex * numMeshVertices, 0.0);
  for (int vtxID = 0; vtxID < numMeshVertices; vtxID++)
  {
    vector<pair<double, int>> sortBuffer(numJointsInfluencingEachVertex);
    for (size_t j = 0; j < weightMatrixEntries[vtxID].size(); j++)
    {
      int frameID = weightMatrixColumnIndices[vtxID][j];
      double weight = weightMatrixEntries[vtxID][j];
      sortBuffer[j] = make_pair(weight, frameID);
    }
    sortBuffer.resize(weightMatrixEntries[vtxID].size());
    assert(sortBuffer.size() > 0);
    sort(sortBuffer.rbegin(), sortBuffer.rend()); // sort in descending order using reverse_iterators
    for(size_t i = 0; i < sortBuffer.size(); i++)
    {
      meshSkinningJoints[vtxID * numJointsInfluencingEachVertex + i] = sortBuffer[i].second;
      meshSkinningWeights[vtxID * numJointsInfluencingEachVertex + i] = sortBuffer[i].first;
    }

    // Note: When the number of joints used on this vertex is smaller than numJointsInfluencingEachVertex,
    // the remaining empty entries are initialized to zero due to vector::assign(XX, 0.0) .
  }
}

void Skinning::applySkinning(const RigidTransform4d * jointSkinTransforms, double * newMeshVertexPositions) const
{
  // Students should implement this

  // The following below is just a dummy implementation.
  for(int vtxID=0; vtxID<numMeshVertices; vtxID++)
  {
    newMeshVertexPositions[3 * vtxID + 0] = 0;
    newMeshVertexPositions[3 * vtxID + 1] = 0;
    newMeshVertexPositions[3 * vtxID + 2] = 0;
    Vec4d restVecHomogeneous = { restMeshVertexPositions[3 * vtxID + 0], restMeshVertexPositions[3 * vtxID + 1], restMeshVertexPositions[3 * vtxID + 2], 1 };
    Vec4d skinVec = { 0, 0, 0, 0 };
    // use meshSkinningJoints and meshSkinningWeights to update weights
    for (int i = 0; i < numJointsInfluencingEachVertex; i++)
    {
        int tempID = meshSkinningJoints[vtxID * numJointsInfluencingEachVertex + i];
        double tempWeight = meshSkinningWeights[vtxID * numJointsInfluencingEachVertex + i];
        
        if (tempWeight > 0.0)
        {
            // the skinVec[3] still be 1, no need to normalize
            skinVec += tempWeight * jointSkinTransforms[tempID] * restVecHomogeneous;
        }
    }
    newMeshVertexPositions[3 * vtxID + 0] = skinVec[0];
    newMeshVertexPositions[3 * vtxID + 1] = skinVec[1];
    newMeshVertexPositions[3 * vtxID + 2] = skinVec[2];
  }
}

void Skinning::applyDualQuaternionSkinning(const RigidTransform4d* jointSkinTransforms, double* newMeshVertexPositions, int numJoints) const
{
    // transform jointSkinTransforms to dual quaternion
    // get rotation matrix and translation vector
    // iterate joints
    std::vector<Dual_quaternion> duals(numJoints);
    for (int i = 0; i < numJoints; i++)
    {
        duals[i] = Dual_quaternion(jointSkinTransforms[i].getRotation(), jointSkinTransforms[i].getTranslation());
    }
    // blend weights
    for (int vtxID = 0; vtxID < numMeshVertices; vtxID++)
    {
        newMeshVertexPositions[3 * vtxID + 0] = 0;
        newMeshVertexPositions[3 * vtxID + 1] = 0;
        newMeshVertexPositions[3 * vtxID + 2] = 0;
        Vec4d restVecHomogeneous = { restMeshVertexPositions[3 * vtxID + 0], restMeshVertexPositions[3 * vtxID + 1], restMeshVertexPositions[3 * vtxID + 2], 1 };
        Vec4d skinVec = { 0, 0, 0, 0 };
        // use meshSkinningJoints and meshSkinningWeights to update weights

        int pivotID = meshSkinningJoints[vtxID * numJointsInfluencingEachVertex];
        Quaternion<double> q0 = duals[pivotID].rotation();
        Dual_quaternion dq_blend = Dual_quaternion();
        for (int i = 0; i < numJointsInfluencingEachVertex; i++)
        {
            int tempID = meshSkinningJoints[vtxID * numJointsInfluencingEachVertex + i];
            double tempWeight = meshSkinningWeights[vtxID * numJointsInfluencingEachVertex + i];

            if (tempWeight > 0.0)
            {
                // flip the sign of dual quaternion, this is to ensure we find two dq with cloest distances
                if (q0.dot(duals[tempID].rotation()) < 0)
                {
                    tempWeight = -tempWeight;
                }
                //skinVec += tempWeight * jointSkinTransforms[tempID] * restVecHomogeneous;
                dq_blend = dq_blend + duals[tempID] * tempWeight;
            }
        }

        // get rotation and tranlation from dual quaternion
        Mat3d rotation;
        Vec3d translation;
        dq_blend.to_transformation(rotation, translation);
        // transform dual quaternion to mat4d
        RigidTransform4d finalTransformation = RigidTransform4d(rotation, translation);
        skinVec = finalTransformation * restVecHomogeneous;

        newMeshVertexPositions[3 * vtxID + 0] = skinVec[0];
        newMeshVertexPositions[3 * vtxID + 1] = skinVec[1];
        newMeshVertexPositions[3 * vtxID + 2] = skinVec[2];
    }
    // transform dual quaternion back to vertex
}

