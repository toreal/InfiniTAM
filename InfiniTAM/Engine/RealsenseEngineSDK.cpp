
#include "RealsenseEngineSDK.h"
//#define COMPILE_WITH_RealSenseSDK 1

#ifdef COMPILE_WITH_RealSenseSDK


#include "pxcsensemanager.h"
#include "pxcprojection.h"
#include "pxccalibration.h"
//#include "util_cmdline.h"
//#include "PXCBlobData.h"

using namespace InfiniTAM::Engine;

#define SDK2016R3 

class RealsenseEngine::PrivateData {
public:
	PrivateData(void) {}


	PXCSenseManager *pp;
	PXCCaptureManager *cm;

#ifndef SDK2016R3

	PXCBlobData* blobData;
	PXCBlobConfiguration* blobConfiguration;
#endif

	pxcStatus sts;
};



RealsenseEngine::RealsenseEngine(const char *calibFilename)//, Vector2i requested_imageSize_rgb, Vector2i requested_imageSize_d)
	: ImageSourceEngine(calibFilename)
{

	data = new PrivateData();
	data->pp = PXCSenseManager::CreateInstance();
	/* Collects command line arguments */
//	UtilCmdLine cmdl(data->pp->QuerySession());
//	pxcCHAR * tmp[] = {L"", L"-csize", L"640x480x30", L"-dsize", L"320x240x60" };
//	cmdl.Parse(L"-listio-nframes-sdname-csize-dsize-isize-lsize-rsize-file-record-noRender-mirror", 5, tmp);
//	if (!cmdl.Parse(L"-listio-nframes-sdname-csize-dsize-isize-lsize-rsize-file-record-noRender-mirror", argc, argv)) return 3;

	/* Sets file recording or playback */
	 data->cm= data->pp->QueryCaptureManager();
	//data->cm->SetFileName(cmdl.m_recordedFile, cmdl.m_bRecord);
	//if (cmdl.m_sdname) 
		//data->cm->FilterByDeviceInfo(cmdl.m_sdname, 0, 0);



	///* Apply command line arguments */
	/*if (cmdl.m_csize.size()>0) {
		data->pp->EnableStream(PXCCapture::STREAM_TYPE_COLOR, cmdl.m_csize.front().first.width, cmdl.m_csize.front().first.height, (pxcF32)cmdl.m_csize.front().second);
	}
	if (cmdl.m_dsize.size()>0) {
		data->pp->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, cmdl.m_dsize.front().first.width, cmdl.m_dsize.front().first.height, (pxcF32)cmdl.m_dsize.front().second);
	}*/
	//if (cmdl.m_isize.size() > 0) {
	//	pp->EnableStream(PXCCapture::STREAM_TYPE_IR, cmdl.m_isize.front().first.width, cmdl.m_isize.front().first.height, (pxcF32)cmdl.m_isize.front().second);
	//}
	//if (cmdl.m_rsize.size() > 0) {
	//	pp->EnableStream(PXCCapture::STREAM_TYPE_RIGHT, cmdl.m_rsize.front().first.width, cmdl.m_rsize.front().first.height, (pxcF32)cmdl.m_rsize.front().second);
	//}
	//if (cmdl.m_lsize.size() > 0) {
	//	pp->EnableStream(PXCCapture::STREAM_TYPE_LEFT, cmdl.m_lsize.front().first.width, cmdl.m_lsize.front().first.height, (pxcF32)cmdl.m_lsize.front().second);
	//}

	//PXCVideoModule::DataDesc desc = {};

	/*if (cmdl.m_csize.size() == 0 && cmdl.m_dsize.size() == 0 && cmdl.m_isize.size() == 0 && cmdl.m_rsize.size() == 0 && cmdl.m_lsize.size() == 0) {
		
		if (data->cm->QueryCapture()) {
			data->cm->QueryCapture()->QueryDeviceInfo(0, &desc.deviceInfo);
		}
		else {
			desc.deviceInfo.streams = PXCCapture::STREAM_TYPE_COLOR | PXCCapture::STREAM_TYPE_DEPTH;
		
		}
		data->pp->EnableStreams(&desc);
	}*/
	//data->pp->EnableStream(PXCCapture::STREAM_TYPE_COLOR, 640, 480, 30);
	//data->pp->EnableStream(PXCCapture::STREAM_TYPE_COLOR, 640, 480, 30);

#ifndef SDK2016R3
	data->pp->EnableBlob(0);	
	PXCBlobModule *blobModule = data->pp->QueryBlob();
	data-> blobData = blobModule->CreateOutput();
	data->blobConfiguration = blobModule->CreateActiveConfiguration();

	data->pp->EnableStream(PXCCapture::StreamType::STREAM_TYPE_COLOR, 640, 480, 30);
	// Enable blob color mapping
	data->blobConfiguration->EnableColorMapping(true);
	data->blobConfiguration->EnableContourExtraction(true);
	data->blobConfiguration->EnableSegmentationImage(true);

	data->blobConfiguration->ApplyChanges();
#endif


	data->pp->EnableStream(PXCCapture::StreamType::STREAM_TYPE_DEPTH, 640, 480, 30);

	/* Initializes the pipeline */
	data->sts = data->pp->Init();
	if (data->sts<PXC_STATUS_NO_ERROR) {
		wprintf_s(L"Failed to locate any video stream(s)\n");
		data->pp->Release();

		return ;
		//return sts;
	}

#ifndef SDK2016R3
	data->blobConfiguration->Update();
#endif

	/* Reset all properties */
	PXCCapture::Device *device = data->pp->QueryCaptureManager()->QueryDevice();
	device->ResetProperties(PXCCapture::STREAM_TYPE_ANY);

	PXCProjection *projection = device->CreateProjection();
	/* Get a calibration instance */


	PXCCalibration *calib = projection-> QueryInstance<PXCCalibration>();


	PXCCalibration::StreamCalibration clibration;
	PXCCalibration::StreamTransform transformation;
	pxcStatus ret= calib->QueryStreamProjectionParameters(PXCCapture::STREAM_TYPE_COLOR, &clibration, &transformation);

	PXCSizeI32 s = data->cm->QueryImageSize(PXCCapture::STREAM_TYPE_COLOR);

	ImageSourceEngine::calib.intrinsics_rgb.SetFrom(clibration.focalLength.x, clibration.focalLength.y, clibration.principalPoint.x, clibration.principalPoint.y,s.width,s.height);
	Matrix4f trans;
	trans.setIdentity();
	trans.m00 = transformation.rotation[0][0];
	trans.m01 = transformation.rotation[0][1];
	trans.m02 = transformation.rotation[0][2];	
	trans.m10 = transformation.rotation[1][0];
	trans.m11 = transformation.rotation[1][1];
	trans.m12 = transformation.rotation[1][2];
	trans.m20 = transformation.rotation[2][0];
	trans.m21 = transformation.rotation[2][1];
	trans.m22 = transformation.rotation[2][2];
	trans.m03 = transformation.translation[0];
	trans.m13 = transformation.translation[1];
	trans.m23 = transformation.translation[2];



	ImageSourceEngine::calib.trafo_rgb_to_depth.SetFrom(trans);

	s = data->cm->QueryImageSize(PXCCapture::STREAM_TYPE_DEPTH);

	 ret = calib->QueryStreamProjectionParameters(PXCCapture::STREAM_TYPE_DEPTH, &clibration, &transformation);
	 ImageSourceEngine::calib.intrinsics_d.SetFrom(clibration.focalLength.x, clibration.focalLength.y, clibration.principalPoint.x, clibration.principalPoint.y, s.width, s.height);

	 /* Release the interface */
		projection->Release();



	/* Set mirror mode */
	//if (cmdl.m_bMirror) {
	//	device->SetMirrorMode(PXCCapture::Device::MirrorMode::MIRROR_MODE_HORIZONTAL);
	//}
	//else {
	//	device->SetMirrorMode(PXCCapture::Device::MirrorMode::MIRROR_MODE_DISABLED);
	//}




}
RealsenseEngine::~RealsenseEngine()
{
}
void RealsenseEngine::getImagesMF(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, MeshFusion * mfdata)
{

	

	data->sts = data->pp->AcquireFrame(false);

	if (data->sts<PXC_STATUS_NO_ERROR) {
		if (data->sts == PXC_STATUS_STREAM_CONFIG_CHANGED) {
			wprintf_s(L"Stream configuration was changed, re-initilizing\n");
			data->pp->Close();
		}
		return;
	}

	/* Render streams, unless -noRender is selected */
//	if (cmdl.m_bNoRender == false) {
		const PXCCapture::Sample *sample = data->pp->QuerySample();
		if (sample) {



			Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);
			PXCImage::ImageData rgbdata;
			PXCImage::ImageInfo rgbinfo = sample->color->QueryInfo();

	
			pxcStatus sts = sample->color->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB24, &rgbdata);
			if (sts >= PXC_STATUS_NO_ERROR)
			{
				pxcBYTE* baseAddr = rgbdata.planes[0];
//				int stride = rgbdata.pitches[0] / sizeof(pxc);
				for (int i = 0; i < rgbImage->noDims.x * rgbImage->noDims.y; i++) {
					Vector4u newPix; char *oldPix = &(((char*)(baseAddr))[i * 3]);
					newPix.x = oldPix[0]; newPix.y = oldPix[1]; newPix.z = oldPix[2]; newPix.w = 255;
					rgb[i] = newPix;
				}


			}

			sample->color->ReleaseAccess(&rgbdata);


			short *depth = rawDepthImage->GetData(MEMORYDEVICE_CPU);
			

//			short invalids[1];
//				invalids[0] = data->pp->QueryCaptureManager()->QueryDevice()->queryde                //QueryDepthSaturationValue();
//				invalids[1] = data->pp->QueryCaptureManager()->QueryDevice()->QueryDepthLowConfidenceValue();
				PXCImage::ImageInfo dinfo = sample->depth->QueryInfo();
				PXCImage::ImageData ddata;
				if (sample->depth->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH,
					& ddata) >= PXC_STATUS_NO_ERROR)
				{
					short *dpixels = (short*)ddata.planes[0];

					memcpy(depth, dpixels, rawDepthImage->dataSize * sizeof(short));


					/*	EnterCriticalSection(&g_depthdataCS);
						memset(g_depthdata, 0, sizeof(g_depthdata));
						int dpitch = ddata.pitches[0] / sizeof(short);
						for (int y = 0; y < (int)dinfo.height; y++)
						{
								for (int x = 0; x < (int)dinfo.width; x++)
								{
										short d = dpixels[y*dpitch + x];
										if (d == invalids[0] || d == invalids[1]) continue;
										g_depthdata[x][y] = d;
								}
						}
						LeaveCriticalSection(&g_depthdataCS);
						g_bDepthdatafilled = true;*/
				}
				sample->depth->ReleaseAccess(&ddata);

#ifndef SDK2016R3
				PXCCapture::Sample * bs = data->pp->QueryBlobSample();

				if (bs)
				{
					data->blobConfiguration->Update();
					data->blobData->Update();
					int n = data->blobData->QueryNumberOfBlobs();

					if (n > 0)
					{
						//data->blobData->QueryBlob()
						PXCBlobData::IBlob* blob=NULL;
						PXCBlobData::SegmentationImageType segmentationType = PXCBlobData::SegmentationImageType::SEGMENTATION_IMAGE_COLOR;
						PXCBlobData::AccessOrderType accessOrder = PXCBlobData::ACCESS_ORDER_LARGE_TO_SMALL;
						 data->blobData->QueryBlob(0, segmentationType, accessOrder, blob);
						 PXCBlobData::IContour * contour;
						 if (blob != NULL)
						 {

							 pxcI32 nc=blob->QueryNumberOfContours();

							 if (nc > 0)
							 {
								 blob->QueryContour(0, contour);

								 pxcI32 np = contour->QuerySize();
								 
								 if (np > 0  && np < mfdata->MAXNODE)
								 {
									 PXCPointI32 * plist = new PXCPointI32[np];
									 mfdata->npoint = np;
									 //mfdata->pointlist = new Vector2i[np];
										 contour->QueryPoints(np, plist);
										 for (int i = 0; i < np; i++)
										 {
											 mfdata->pointlist[i].x = plist[i].x;
											 mfdata->pointlist[i].y = plist[i].y;

										 }


								 }
								 else
									 mfdata->npoint = 0;
								 PXCImage* segimg=NULL;


								 blob->QuerySegmentationImage(segimg);
								 if (segimg != NULL)
								 {
									 Vector4u *rgb = mfdata->segImage->GetData(MEMORYDEVICE_CPU);
									 PXCImage::ImageData rgbdata;
									 PXCImage::ImageInfo rgbinfo = segimg->QueryInfo();


									 pxcStatus sts = segimg->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB24, &rgbdata);
									 if (sts >= PXC_STATUS_NO_ERROR)
									 {
										 pxcBYTE* baseAddr = rgbdata.planes[0];
										 //				int stride = rgbdata.pitches[0] / sizeof(pxc);
										 for (int i = 0; i < rgbImage->noDims.x * rgbImage->noDims.y; i++) {
											 Vector4u newPix; char *oldPix = &(((char*)(baseAddr))[i * 3]);
											 newPix.x = oldPix[0]; newPix.y = oldPix[1]; newPix.z = oldPix[2]; newPix.w = 255;
											 rgb[i] = newPix;
										 }


									 }

									 segimg->ReleaseAccess(&rgbdata);

								 }
							 }
						 }


					}


				}
#endif

		/*	if (sample->depth && !renderd.RenderFrame(sample->depth)) break;
			if (sample->color && !renderc.RenderFrame(sample->color)) break;
			if (sample->ir    && !renderi.RenderFrame(sample->ir))    break;
			if (sample->right    && !renderr.RenderFrame(sample->right))    break;
			if (sample->left    && !renderl.RenderFrame(sample->left))    break;*/
		}
//	}
	/* Releases lock so pipeline can process next frame */
	data->pp->ReleaseFrame();




	return;
}
bool RealsenseEngine::hasMoreImages(void)
{
	return true;
}
Vector2i RealsenseEngine::getDepthImageSize(void)
{
	try {
		if (data->sts<PXC_STATUS_NO_ERROR)
			return Vector2i(0, 0);

		PXCSizeI32 s = data->cm->QueryImageSize(PXCCapture::STREAM_TYPE_DEPTH);
		if (&s != NULL && s.width >= 0 && s.height >= 0)
			return Vector2i(s.width, s.height);
		else
			return Vector2i(0, 0);

	}
	catch (std::exception &em)
	{
		return Vector2i(0, 0);
	}

	
	
}
Vector2i RealsenseEngine::getRGBImageSize(void)
{
	try {
		if (data->sts<PXC_STATUS_NO_ERROR)
			return Vector2i(0, 0);

	PXCSizeI32 s = data->cm->QueryImageSize(PXCCapture::STREAM_TYPE_COLOR);
	if (&s != NULL && s.width >= 0 && s.height >= 0)
		return Vector2i(s.width, s.height);
	else
		return Vector2i(0, 0);
	}
	catch (std::exception &em)
	{
	return Vector2i(0, 0);
	}
}
#else

using namespace InfiniTAM::Engine;

RealsenseEngine::RealsenseEngine(const char *calibFilename)//, Vector2i requested_imageSize_rgb, Vector2i requested_imageSize_d)
	: ImageSourceEngine(calibFilename)
{
	printf("compiled without RealSense Windows support\n");
}
RealsenseEngine::~RealsenseEngine()
{}
void RealsenseEngine::getImagesMF(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, MeshFusion * data)
{
	return;
}
bool RealsenseEngine::hasMoreImages(void)
{
	return false;
}
Vector2i RealsenseEngine::getDepthImageSize(void)
{
	return Vector2i(0, 0);
}
Vector2i RealsenseEngine::getRGBImageSize(void)
{
	return Vector2i(0, 0);
}


#endif
