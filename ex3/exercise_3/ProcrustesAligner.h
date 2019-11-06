#pragma once
#include "SimpleMesh.h"
#include <iostream>

class ProcrustesAligner {
public:
	Matrix4f estimatePose(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
		ASSERT(sourcePoints.size() == targetPoints.size() && "The number of source and target points should be the same, since every source point is matched with corresponding target point.");

		// We estimate the pose between source and target points using Procrustes algorithm.
		// Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
		// from source points to target points.

		auto sourceMean = computeMean(sourcePoints);
		auto targetMean = computeMean(targetPoints);
		
		Matrix3f rotation = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
		Vector3f translation = computeTranslation(sourceMean, targetMean);
		Vector3f finalTranslation = -rotation * sourceMean + translation + sourceMean;

		// Compute the transformation matrix by using the computed rotation and translation.
		// Note: The final translation is not equal to the translation of the means. Refer to the exercise sheet for more details.
		// You can access parts of the matrix with .block(start_row, start_col, num_rows, num_cols) = elements
		Matrix4f estimatedPose = Matrix4f::Identity();		
		estimatedPose.block(0, 0, 3, 3) = rotation;
		estimatedPose.block(0, 3, 3, 1) = finalTranslation;

		return estimatedPose;
	}

private:
	Vector3f computeMean(const std::vector<Vector3f>& points) {
		// Compute the mean of input points.
		float meanX = 0, meanY = 0, meanZ = 0;
		unsigned len = points.size();
		
		for (unsigned i = 0; i < len; i++) {
			meanX += points[i](0);
			meanY += points[i](1);
			meanZ += points[i](2);
		}

		meanX = meanX / len;
		meanY = meanY / len;
		meanZ = meanZ / len;

		Vector3f mean = Vector3f(meanX, meanY, meanZ);
		return mean;
	}

	Matrix3f estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
		// Estimate the rotation from source to target points, following the Procrustes algorithm.
		// To compute the singular value decomposition you can use JacobiSVD() from Eigen.
		// Important: The covariance matrices should contain mean-centered source/target points.
		Matrix3f rotation = Matrix3f::Identity();
		unsigned N = sourcePoints.size();
		MatrixXf X(N, 3);
		MatrixXf Xprime(N, 3);

		X.setZero();
		Xprime.setZero();

		// center both source and target points and compute cross-covariance matrix
		for (unsigned i = 0; i < N; i++) {
			Xprime(i, 0) = sourcePoints[i](0) - sourceMean(0);
			Xprime(i, 1) = sourcePoints[i](1) - sourceMean(1);
			Xprime(i, 2) = sourcePoints[i](2) - sourceMean(2);

			X(i, 0) = targetPoints[i](0) - targetMean(0);
			X(i, 1) = targetPoints[i](1) - targetMean(1);
			X(i, 2) = targetPoints[i](2) - targetMean(2);
		}

		auto cov = X.transpose() * Xprime;
		
		// perform SVD to get U, S and V
		JacobiSVD<MatrixXf> svd(cov, ComputeFullU | ComputeFullV);

		// compute R = U * V'
		Matrix3f R = svd.matrixU() * svd.matrixV().transpose();

		return R;
	}

	Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean) {
		// Compute the translation vector from source to target points.
		// Vector3f translation = Vector3f::Zero();
		return targetMean - sourceMean;
	}
};