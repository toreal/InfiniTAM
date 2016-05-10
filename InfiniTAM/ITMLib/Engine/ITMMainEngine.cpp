// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMMainEngine.h"
#include "../../Utils/FileUtils.h"
#include "../MeshFusion/Tracker.h"
#include "../../Utils/NVTimer.h"

using namespace ITMLib::Engine;

bool bsence = false;

ITMMainEngine::ITMMainEngine(const ITMLibSettings *settings, const ITMRGBDCalib *calib, Vector2i imgSize_rgb, Vector2i imgSize_d)
{
	// create all the things required for marching cubes and mesh extraction
	// - uses additional memory (lots!)
	static const bool createMeshingEngine = true;

	if ((imgSize_d.x == -1) || (imgSize_d.y == -1)) imgSize_d = imgSize_rgb;

	this->settings = settings;

	meshingEngine = NULL;

	if (bsence)
	{
		this->scene = new ITMScene<ITMVoxel, ITMVoxelIndex>(&(settings->sceneParams), settings->useSwapping,
			settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU);

		switch (settings->deviceType)
		{
		case ITMLibSettings::DEVICE_CPU:
			lowLevelEngine = new ITMLowLevelEngine_CPU();
			viewBuilder = new ITMViewBuilder_CPU(calib);
			visualisationEngine = new ITMVisualisationEngine_CPU<ITMVoxel, ITMVoxelIndex>(scene);
			if (createMeshingEngine) meshingEngine = new ITMMeshingEngine_CPU<ITMVoxel, ITMVoxelIndex>();
			break;
		case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
			lowLevelEngine = new ITMLowLevelEngine_CUDA();
			viewBuilder = new ITMViewBuilder_CUDA(calib);
			visualisationEngine = new ITMVisualisationEngine_CUDA<ITMVoxel, ITMVoxelIndex>(scene);
			if (createMeshingEngine) meshingEngine = new ITMMeshingEngine_CUDA<ITMVoxel, ITMVoxelIndex>();
#endif
			break;
		case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
			lowLevelEngine = new ITMLowLevelEngine_Metal();
			viewBuilder = new ITMViewBuilder_Metal(calib);
			visualisationEngine = new ITMVisualisationEngine_Metal<ITMVoxel, ITMVoxelIndex>(scene);
			if (createMeshingEngine) meshingEngine = new ITMMeshingEngine_CPU<ITMVoxel, ITMVoxelIndex>();
#endif
			break;
		}


		mesh = NULL;
		if (createMeshingEngine) mesh = new ITMMesh(settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU);

		Vector2i trackedImageSize = ITMTrackingController::GetTrackedImageSize(settings, imgSize_rgb, imgSize_d);

		renderState_live = visualisationEngine->CreateRenderState(trackedImageSize);
		renderState_freeview = NULL; //will be created by the visualisation engine

		denseMapper = new ITMDenseMapper<ITMVoxel, ITMVoxelIndex>(settings);
		denseMapper->ResetScene(scene);

		imuCalibrator = new ITMIMUCalibrator_iPad();
		tracker = ITMTrackerFactory<ITMVoxel, ITMVoxelIndex>::Instance().Make(trackedImageSize, settings, lowLevelEngine, imuCalibrator, scene);
		trackingController = new ITMTrackingController(tracker, visualisationEngine, lowLevelEngine, settings);

		trackingState = trackingController->BuildTrackingState(trackedImageSize);
		tracker->UpdateInitialPose(trackingState);
	}
	else
	{

		viewBuilder = new ITMViewBuilder_CPU(calib);
		mesh = NULL;
		if (createMeshingEngine) mesh = new ITMMesh(settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU);

		trackingState =new ITMTrackingState(Vector2i(0,0) , MEMORYDEVICE_CPU);


	}
	view = NULL; // will be allocated by the view builder

	fusionActive = true;
	mainProcessingActive = true;
//	mfdata = new MeshFusion();
}

ITMMainEngine::~ITMMainEngine()
{
	if (renderState_live!=NULL)
	delete renderState_live;
	if (renderState_freeview!=NULL) delete renderState_freeview;

	delete scene;

	delete denseMapper;
	delete trackingController;

	delete tracker;
	delete imuCalibrator;

	delete lowLevelEngine;
	delete viewBuilder;

	delete trackingState;
	if (view != NULL) delete view;

	delete visualisationEngine;

	if (meshingEngine != NULL) delete meshingEngine;

	if (mesh != NULL) delete mesh;


	//if (mfdata != NULL) delete mfdata;
}

ITMMesh* ITMMainEngine::UpdateMesh(void)
{
	if (mesh != NULL) meshingEngine->MeshScene(mesh, scene);
	return mesh;
}

void ITMMainEngine::SaveSceneToMesh(const char *objFileName)
{
	if (mesh == NULL) return;
	if (bsence)
	{
		meshingEngine->MeshScene(mesh, scene);
		mesh->WriteSTL(objFileName);
	}
	else
	{
		mesh->WriteSTL(objFileName);
		mesh->WriteOBJ("m.obj");
	}
}

void ITMMainEngine::ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMIMUMeasurement *imuMeasurement)
{
	StopWatchInterface *timer_instant;
	sdkCreateTimer(&timer_instant);
	sdkResetTimer(&timer_instant);
	sdkStartTimer(&timer_instant);
	sdkStopTimer(&timer_instant);
	float processedTime_inst = sdkGetTimerValue(&timer_instant);
	std::cout << "Time:" << processedTime_inst << std::endl;
	sdkResetTimer(&timer_instant);
	sdkStartTimer(&timer_instant);
	// prepare image and turn it into a depth image
	if (imuMeasurement == NULL) viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, settings->modelSensorNoise);
	else viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, imuMeasurement);

	if (!mainProcessingActive) return;

	mfdata->mainView = view;


	//SaveImageToFile(view->depthNormal, "normal.ppm");
	//SaveImageToFile(view->curvature, "curvature.ppm");

	mfdata->buildProjDepth();

	float mindis;
	
	//M
	assert(view->rgb != NULL);
	if (view->rgb)
		mfdata->MeshFusion_Tracking(mindis);

	sdkStopTimer(&timer_instant);
	 processedTime_inst = sdkGetTimerValue(&timer_instant);
	std::cout << "tracking Time:" << processedTime_inst << std::endl;
	sdkResetTimer(&timer_instant);
	sdkStartTimer(&timer_instant);
	//std::cout << mindis << std::endl;

	if ( mesh->noTotalTriangles==0 || mindis > 100 )
	{
		try {
			mesh->noTotalTriangles = 0;
			//build silhouette features
			mfdata->sortpoint(view->rgb);
			//update current pose
			mfdata->estimatePose(this->trackingState->pose_d);

			sdkStopTimer(&timer_instant);
			 processedTime_inst = sdkGetTimerValue(&timer_instant);
			std::cout << "pose Time:" << processedTime_inst << std::endl;
			sdkResetTimer(&timer_instant);
			sdkStartTimer(&timer_instant);
			if (! mfdata->bmesh)
			mfdata->constructMesh(mesh);
			
			mfdata->meshUpdate(mesh, this->trackingState->pose_d);

			sdkStopTimer(&timer_instant);
			 processedTime_inst = sdkGetTimerValue(&timer_instant);
			std::cout << " update mesh Time:" << processedTime_inst << std::endl;
			sdkResetTimer(&timer_instant);
			sdkStartTimer(&timer_instant);
			if (mindis > 5)
			{
				mfdata->MeshFusion_InitTracking();
				if (view->rgb)
					mfdata->MeshFusion_Tracking(mindis);


			}
			mfdata->Generate3DPoints();

			sdkStopTimer(&timer_instant);
			 processedTime_inst = sdkGetTimerValue(&timer_instant);
			std::cout << "generate 3d point Time:" << processedTime_inst << std::endl;
			sdkResetTimer(&timer_instant);
		}
		catch (std::exception em)
		{
			std::cout << em.what();

		}
	}

	

	


	if (bsence)
	{
	
	//// tracking
	trackingController->Track(trackingState, view);

	//// fusion
	if (fusionActive) denseMapper->ProcessFrame(view, trackingState, scene, renderState_live);

	//// raycast to renderState_live for tracking and free visualisation
	trackingController->Prepare(trackingState, view, renderState_live);
     }
}

Vector2i ITMMainEngine::GetImageSize(void) const
{
	return renderState_live->raycastImage->noDims;
}

void ITMMainEngine::GetImage(ITMUChar4Image *out, GetImageType getImageType, ITMPose *pose, ITMIntrinsics *intrinsics)
{
	if (view == NULL ) return;

	out->Clear();

	switch (getImageType)
	{
	case ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB:
		out->ChangeDims(view->rgb->noDims);
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) 
			out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		break;
	case ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH:
		out->ChangeDims(view->depth->noDims);
		if (settings->trackerType==ITMLib::Objects::ITMLibSettings::TRACKER_WICP)
		{
			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->depthUncertainty->UpdateHostFromDevice();
			ITMVisualisationEngine<ITMVoxel, ITMVoxelIndex>::WeightToUchar4(out, view->depthUncertainty);
		}
		else
		{
			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->depth->UpdateHostFromDevice();
			ITMVisualisationEngine<ITMVoxel, ITMVoxelIndex>::DepthToUchar4(out, view->depth);
		}

		break;
	case ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST:
	{
		if (bsence)
		{
			ORUtils::Image<Vector4u> *srcImage = renderState_live->raycastImage;
			out->ChangeDims(srcImage->noDims);
			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
				out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
			else out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		}
		break;
	}
	case ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED:
	case ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME:
	case ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL:
	{
		if (bsence)
		{
			IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;
			if (getImageType == ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME) type = IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME;
			else if (getImageType == ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL) type = IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL;
			if (renderState_freeview == NULL) renderState_freeview = visualisationEngine->CreateRenderState(out->noDims);

			visualisationEngine->FindVisibleBlocks(pose, intrinsics, renderState_freeview);
			visualisationEngine->CreateExpectedDepths(pose, intrinsics, renderState_freeview);
			visualisationEngine->RenderImage(pose, intrinsics, renderState_freeview, renderState_freeview->raycastImage, type);

			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
				out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
			else out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		}
		break;
	}
	case ITMMainEngine::InfiniTAM_IMAGE_UNKNOWN:
		break;
	};
}

void ITMMainEngine::turnOnIntegration() { fusionActive = true; }
void ITMMainEngine::turnOffIntegration() { fusionActive = false; }
void ITMMainEngine::turnOnMainProcessing() { mainProcessingActive = true; }
void ITMMainEngine::turnOffMainProcessing() { mainProcessingActive = false; }
