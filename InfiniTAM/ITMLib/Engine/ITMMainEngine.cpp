// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMMainEngine.h"
#include "../../Utils/FileUtils.h"
#include "../MeshFusion/Tracker.h"
#include "../../Utils/NVTimer.h"

using namespace ITMLib::Engine;

bool bsence = false;
extern bool bdomf;

#define OUTPUT 0

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
		mesh->WriteOBJ("m.obj");
	}
	else
	{
		mesh->WriteSTL(objFileName);
		mesh->WriteOBJ("m.obj");
	}
}

void ITMMainEngine::ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage,int currentFrameNo, ITMIMUMeasurement *imuMeasurement)
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

	if (bdomf)
	{
		mfdata->mainView = view;


		mfdata->buildProjDepth();

		mfdata->NormalAndCurvature(&view, settings->modelSensorNoise);

	//	mfdata->label(&view);
	}
	//SaveImageToFile(view->depthNormal, "normal.ppm");
	//SaveImageToFile(view->curvature, "curvature.ppm");



	float mindis;
	
	//M
	assert(view->rgb != NULL);
	if ( view->rgb)
		mfdata->MeshFusion_Tracking(mindis, currentFrameNo);

	sdkStopTimer(&timer_instant);
	 processedTime_inst = sdkGetTimerValue(&timer_instant);
	std::cout << "tracking Time:" << processedTime_inst << std::endl;
	sdkResetTimer(&timer_instant);
	sdkStartTimer(&timer_instant);
	//std::cout << mindis << std::endl;

	if ( mfdata->mytriData.totalFace == 0 || mindis > 400)
	{
		//if (false)
		{
		try {
			//	mesh->noTotalTriangles = 0;
				//build silhouette features
				//mfdata->sortpoint(view->rgb);
				//update current pose
			//	mfdata->estimatePose(this->trackingState->pose_d);

			sdkStopTimer(&timer_instant);
			processedTime_inst = sdkGetTimerValue(&timer_instant);
			std::cout << "pose Time:" << processedTime_inst << std::endl;
			sdkResetTimer(&timer_instant);
			sdkStartTimer(&timer_instant);


			sdkStopTimer(&timer_instant);
			processedTime_inst = sdkGetTimerValue(&timer_instant);
			std::cout << " update mesh Time:" << processedTime_inst << std::endl;
			sdkResetTimer(&timer_instant);
			sdkStartTimer(&timer_instant);
			if (mindis > 5)
			{
				mfdata->MeshFusion_InitTracking();
				if (view->rgb)
					mfdata->MeshFusion_Tracking(mindis, currentFrameNo);

			}
			mfdata->sortpoint(view->rgb);



			int h = view->depthNormal->noDims.height;
			int w = view->depthNormal->noDims.width;

			cv::Mat buf = cv::Mat(h, w, CV_8SC3);



		//	mfdata->constructMesh(mesh, &mfdata->currTri);
		//	mfdata->currTri.buildHalfEdge(mfdata);
			mfdata->label(&view , buf, currentFrameNo);
		//	mfdata->makeTri(mesh, &mfdata->currTri);
			mfdata->ds.makeTri(mfdata->depth3d, buf, view->calib->intrinsics_rgb.projectionParamsSimple.all);

			if (!mfdata->bmesh)
			{
		//		mfdata->mytriData.copyFrom(&mfdata->currTri);
				mfdata->bmesh = true;
			}
			else
			{
				mfdata->Generate3DPoints(mfdata->m_backup, mfdata->newPoints);
#ifdef OUTPUT
				FILE * fw = fopen("result1.txt", "w");
				for (int i = 0; i < mfdata->objectPoints.size(); i++)
				{
					cv::Point3f p = mfdata->objectPoints[i];
					fprintf(fw, "%f, %f, %f\n", p.x, p.y, p.z);

				}
				fclose(fw);


				fw = fopen("result2.txt", "w");
				for (int i = 0; i < mfdata->newPoints.size(); i++)
				{
					cv::Point3f p = mfdata->newPoints[i];
					fprintf(fw, "%f, %f, %f\n", p.x, p.y, p.z);

				}
				fclose(fw);

#endif
				//pnp 估pose,目前並沒有更新到pose_d,所以有作沒作都一樣
				//mfdata->estimatePose(trackingState->pose_d);
				//mfdata->refinePose(trackingState->pose_d);
				//mfdata->goodFeature(trackingState->pose_d);
				//mfdata->ReCoordinateSystem(trackingState->pose_d);
				mfdata->rotateAngle(trackingState->pose_d);


				//已不適合呼叫
				//	mfdata->meshUpdate(mesh, this->trackingState->pose_d, &mfdata->mytriData);

				//新的mesh merge,但當點的位置有錯,會有問題,所以先不作
				//	mfdata->meshMerge(mesh, this->trackingState->pose_d, &mfdata->mytriData);
			}
			mfdata->Generate3DPoints(mfdata->m_base_corners, mfdata->objectPoints);

			sdkStopTimer(&timer_instant);
			processedTime_inst = sdkGetTimerValue(&timer_instant);
			std::cout << "generate 3d point Time:" << processedTime_inst << std::endl;
			std::cout << mfdata->m_base_corners.size() <<"," << mfdata->m_backup.size()<< endl;
			sdkResetTimer(&timer_instant);
		}
		catch (std::exception em)
		{
			std::cout << em.what();

		}

	}/*else		
	{
	
			mfdata->rotateAngle(trackingState->pose_d);
	

	}*/


		if (bsence)
		{


		//	SaveImageToFile(mfdata->proDepth, "prodepth.ppm");
		//	//// tracking
		////	trackingController->Track(trackingState, view);

		//	float* dd = view->depth->GetData(MEMORYDEVICE_CPU);
		//	int xlens = view->depth->noDims.x;
		//	int ylens = view->depth->noDims.y;

		//	float * rgbd = mfdata->proDepth->GetData(MEMORYDEVICE_CPU);
		//	for (int nx = 0; nx < xlens; nx++)
		//		for (int ny = 0; ny < ylens; ny++)
		//		{
		//			float pz = rgbd[nx + ny*xlens] ;
		//			dd[nx + ny*xlens] = pz;
		//		}
		//		



			cout << "R" <<trackingState->pose_d->GetR() << endl;
			cout << "T" << trackingState->pose_d->GetT() << endl;

			//// fusion
			if (fusionActive) 
				denseMapper->ProcessFrame(view, trackingState, scene, renderState_live);

			//// raycast to renderState_live for tracking and free visualisation
			trackingController->Prepare(trackingState, view, renderState_live);
		}


	}//end of if 

}//end of function

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
