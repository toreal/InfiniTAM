// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "MFDepthTracker.h"
#include "../Engine/DeviceAgnostic/ITMDepthTracker.h"
#include "../../ORUtils/Cholesky.h"

#include <math.h>

using namespace ITMLib::Engine;

MFDepthTracker::MFDepthTracker()//Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel, float distThresh,
	//float terminationThreshold, const ITMLowLevelEngine *lowLevelEngine, MemoryDeviceType memoryType)
{
	//viewHierarchy = new ITMImageHierarchy<ITMTemplatedHierarchyLevel<ITMFloatImage> >(imgSize, trackingRegime, noHierarchyLevels, memoryType, true);
	//sceneHierarchy = new ITMImageHierarchy<ITMSceneHierarchyLevel>(imgSize, trackingRegime, noHierarchyLevels, memoryType, true);

	//this->noIterationsPerLevel = new int[noHierarchyLevels];
	//this->distThresh = new float[noHierarchyLevels];
	//
	this->noIterationsPerLevel = 2; //TODO -> make parameter
	//this->noIterationsPerLevel[0] = 2; //TODO -> make parameter
	//for (int levelId = 1; levelId < noHierarchyLevels; levelId++)
	//{
	//	noIterationsPerLevel[levelId] = noIterationsPerLevel[levelId - 1] + 2;
	//}

	//float distThreshStep = distThresh / noHierarchyLevels;
	//this->distThresh[noHierarchyLevels - 1] = distThresh;
	//for (int levelId = noHierarchyLevels - 2; levelId >= 0; levelId--)
	//	this->distThresh[levelId] = this->distThresh[levelId + 1] - distThreshStep;

	//this->lowLevelEngine = lowLevelEngine;

	//this->noICPLevel = noICPRunTillLevel;

	//this->terminationThreshold = terminationThreshold;
	iterationType = TRACKER_ITERATION_BOTH;
}

MFDepthTracker::~MFDepthTracker(void) 
{ 
//	delete this->viewHierarchy;
//	delete this->sceneHierarchy;

//	delete[] this->noIterationsPerLevel;
//	delete[] this->distThresh;
}

void MFDepthTracker::SetEvaluationData(ITMPose *pose_d, const ITMView *view)
{
	this->pose = pose_d;
	this->view = view;

	//sceneHierarchy->levels[0]->intrinsics = view->calib->intrinsics_d.projectionParamsSimple.all;
	//viewHierarchy->levels[0]->intrinsics = view->calib->intrinsics_d.projectionParamsSimple.all;

	// the image hierarchy allows pointers to external data at level 0
	//viewHierarchy->levels[0]->depth = view->depth;
	//sceneHierarchy->levels[0]->pointsMap = trackingState->pointCloud->locations;
	//sceneHierarchy->levels[0]->normalsMap = trackingState->pointCloud->colours;

	//scenePose = trackingState->pose_pointCloud->GetM();
}

void MFDepthTracker::PrepareForEvaluation()
{
	//for (int i = 1; i < viewHierarchy->noLevels; i++)
	{
	//	ITMTemplatedHierarchyLevel<ITMFloatImage> *currentLevelView = viewHierarchy->levels[i], *previousLevelView = viewHierarchy->levels[i - 1];
	//	lowLevelEngine->FilterSubsampleWithHoles(currentLevelView->depth, previousLevelView->depth);
	//	currentLevelView->intrinsics = previousLevelView->intrinsics * 0.5f;

	//	ITMSceneHierarchyLevel *currentLevelScene = sceneHierarchy->levels[i], *previousLevelScene = sceneHierarchy->levels[i - 1];
		//lowLevelEngine->FilterSubsampleWithHoles(currentLevelScene->pointsMap, previousLevelScene->pointsMap);
		//lowLevelEngine->FilterSubsampleWithHoles(currentLevelScene->normalsMap, previousLevelScene->normalsMap);
	//	currentLevelScene->intrinsics = previousLevelScene->intrinsics * 0.5f;
	}
}

void MFDepthTracker::SetEvaluationParams(int levelId)
{
	this->levelId = levelId;
//	this->iterationType = viewHierarchy->levels[levelId]->iterationType;
//	this->sceneHierarchyLevel = sceneHierarchy->levels[0];
//	this->viewHierarchyLevel = viewHierarchy->levels[levelId];
}

void MFDepthTracker::ComputeDelta(float *step, float *nabla, float *hessian, bool shortIteration) const
{
	for (int i = 0; i < 6; i++) step[i] = 0;

	if (shortIteration)
	{
		float smallHessian[3 * 3];
		for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) smallHessian[r + c * 3] = hessian[r + c * 6];

		ORUtils::Cholesky cholA(smallHessian, 3);
		cholA.Backsub(step, nabla);
	}
	else
	{
		ORUtils::Cholesky cholA(hessian, 6);
		cholA.Backsub(step, nabla);
	}
}

bool MFDepthTracker::HasConverged(float *step) const
{
	float stepLength = 0.0f;
	for (int i = 0; i < 6; i++) stepLength += step[i] * step[i];

	if (sqrt(stepLength) / 6 < terminationThreshold) return true; //converged

	return false;
}

void MFDepthTracker::ApplyDelta(const Matrix4f & para_old, const float *delta, Matrix4f & para_new) const
{
	float step[6];

	switch (iterationType)
	{
	case TRACKER_ITERATION_ROTATION:
		step[0] = (float)(delta[0]); step[1] = (float)(delta[1]); step[2] = (float)(delta[2]);
		step[3] = 0.0f; step[4] = 0.0f; step[5] = 0.0f;
		break;
	case TRACKER_ITERATION_TRANSLATION:
		step[0] = 0.0f; step[1] = 0.0f; step[2] = 0.0f;
		step[3] = (float)(delta[0]); step[4] = (float)(delta[1]); step[5] = (float)(delta[2]);
		break;
	default:
	case TRACKER_ITERATION_BOTH:
		step[0] = (float)(delta[0]); step[1] = (float)(delta[1]); step[2] = (float)(delta[2]);
		step[3] = (float)(delta[3]); step[4] = (float)(delta[4]); step[5] = (float)(delta[5]);
		break;
	}

	Matrix4f Tinc;

	Tinc.m00 = 1.0f;		Tinc.m10 = step[2];		Tinc.m20 = -step[1];	Tinc.m30 = step[3];
	Tinc.m01 = -step[2];	Tinc.m11 = 1.0f;		Tinc.m21 = step[0];		Tinc.m31 = step[4];
	Tinc.m02 = step[1];		Tinc.m12 = -step[0];	Tinc.m22 = 1.0f;		Tinc.m32 = step[5];
	Tinc.m03 = 0.0f;		Tinc.m13 = 0.0f;		Tinc.m23 = 0.0f;		Tinc.m33 = 1.0f;

	para_new = Tinc * para_old;
}

void MFDepthTracker::MyTrackCamera(ITMPose *pose_d, const ITMView *view)
{
	this->SetEvaluationData(pose_d, view);
	this->PrepareForEvaluation();

	float f_old = 1e10, f_new;
	int noValidPoints_new;

	float hessian_good[6 * 6], hessian_new[6 * 6], A[6 * 6];
	float nabla_good[6], nabla_new[6];
	float step[6];
	int levelId = 1;
	//for (int levelId = viewHierarchy->noLevels - 1; levelId >= noICPLevel; levelId--)
	{
		this->SetEvaluationParams(levelId);
		//if (iterationType == TRACKER_ITERATION_NONE) continue;

		Matrix4f approxInvPose = pose_d->GetInvM();
		ITMPose lastKnownGoodPose(*(pose_d));
		f_old = 1e20f;
		float lambda = 1.0;

		for (int iterNo = 0; iterNo < noIterationsPerLevel; iterNo++)
		{
			// evaluate error function and gradients
			noValidPoints_new = this->ComputeGandH(f_new, nabla_new, hessian_new, approxInvPose);

			// check if error increased. If so, revert
			if ((noValidPoints_new <= 0)||(f_new > f_old)) {
				pose_d->SetFrom(&lastKnownGoodPose);
				approxInvPose =pose_d->GetInvM();
				lambda *= 10.0f;
			} else {
				lastKnownGoodPose.SetFrom(pose_d);
				f_old = f_new;

				for (int i = 0; i < 6*6; ++i) hessian_good[i] = hessian_new[i] / noValidPoints_new;
				for (int i = 0; i < 6; ++i) nabla_good[i] = nabla_new[i] / noValidPoints_new;
				lambda /= 10.0f;
			}
			for (int i = 0; i < 6*6; ++i) A[i] = hessian_good[i];
			for (int i = 0; i < 6; ++i) A[i+i*6] *= 1.0f + lambda;

			// compute a new step and make sure we've got an SE3
			ComputeDelta(step, nabla_good, A, iterationType != TRACKER_ITERATION_BOTH);
			ApplyDelta(approxInvPose, step, approxInvPose);
			pose_d->SetInvM(approxInvPose);
			pose_d->Coerce();
			approxInvPose = pose_d->GetInvM();

			// if step is small, assume it's going to decrease the error and finish
			if (HasConverged(step)) break;
		}
	}
}



int MFDepthTracker::ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose)
{
	Vector4f *pointsMap =NULL;//;// = sceneHierarchyLevel->pointsMap->GetData(MEMORYDEVICE_CPU);
	Vector4f *normalsMap =NULL;// = sceneHierarchyLevel->normalsMap->GetData(MEMORYDEVICE_CPU);
	Vector4f sceneIntrinsics;// = sceneHierarchyLevel->intrinsics;
	Vector2i sceneImageSize;// = sceneHierarchyLevel->pointsMap->noDims;

	float *depth;// = viewHierarchyLevel->depth->GetData(MEMORYDEVICE_CPU);
	Vector4f viewIntrinsics;// = viewHierarchyLevel->intrinsics;
	Vector2i viewImageSize;// = viewHierarchyLevel->depth->noDims;

	if (iterationType == TRACKER_ITERATION_NONE) return 0;

	bool shortIteration = (iterationType == TRACKER_ITERATION_ROTATION) || (iterationType == TRACKER_ITERATION_TRANSLATION);

	float sumHessian[6 * 6], sumNabla[6], sumF; int noValidPoints;
	int noPara = shortIteration ? 3 : 6, noParaSQ = shortIteration ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;

	noValidPoints = 0; sumF = 0.0f;
	memset(sumHessian, 0, sizeof(float) * noParaSQ);
	memset(sumNabla, 0, sizeof(float) * noPara);

	for (int y = 0; y < viewImageSize.y; y++) for (int x = 0; x < viewImageSize.x; x++)
	{
		float localHessian[6 + 5 + 4 + 3 + 2 + 1], localNabla[6], localF = 0;

		for (int i = 0; i < noPara; i++) localNabla[i] = 0.0f;
		for (int i = 0; i < noParaSQ; i++) localHessian[i] = 0.0f;

		bool isValidPoint;

		switch (iterationType)
		{
		case TRACKER_ITERATION_ROTATION:
			isValidPoint = computePerPointGH_Depth<true, true>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], viewImageSize,
				viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, distThresh);
			break;
		case TRACKER_ITERATION_TRANSLATION:
			isValidPoint = computePerPointGH_Depth<true, false>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], viewImageSize,
				viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, distThresh);
			break;
		case TRACKER_ITERATION_BOTH:
			isValidPoint = computePerPointGH_Depth<false, false>(localNabla, localHessian, localF, x, y, 0, viewImageSize,
				viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, distThresh);
			break;
		default:
			isValidPoint = false;
			break;
		}

		if (isValidPoint)
		{
			noValidPoints++; sumF += localF;
			for (int i = 0; i < noPara; i++) sumNabla[i] += localNabla[i];
			for (int i = 0; i < noParaSQ; i++) sumHessian[i] += localHessian[i];
		}
	}

	for (int r = 0, counter = 0; r < noPara; r++) for (int c = 0; c <= r; c++, counter++) hessian[r + c * 6] = sumHessian[counter];
	for (int r = 0; r < noPara; ++r) for (int c = r + 1; c < noPara; c++) hessian[r + c * 6] = hessian[c + r * 6];

	memcpy(nabla, sumNabla, noPara * sizeof(float));
	f = (noValidPoints > 100) ? sqrt(sumF) / noValidPoints : 1e5f;

	return noValidPoints;
}




