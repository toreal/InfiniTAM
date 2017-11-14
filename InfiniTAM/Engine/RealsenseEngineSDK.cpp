
#include "RealsenseEngineSDK.h"
#define COMPILE_WITH_RealSenseSDK 1
#define SDK20 1

#ifdef COMPILE_WITH_RealSenseSDK

#ifdef SDK20
#include <librealsense2/rs.hpp>


#else

#include "pxcsensemanager.h"
#include "pxcprojection.h"
#include "pxccalibration.h"

#endif
using namespace InfiniTAM::Engine;

#define SDK2016R3 

class RealsenseEngine::PrivateData {
public:
	PrivateData(void) {}

#ifndef SDK20
	PXCSenseManager *pp;
	PXCCaptureManager *cm;
	pxcStatus sts;
#else
	rs2::pipeline pipe;
	rs2::frameset data;

#endif
#ifndef SDK2016R3

	PXCBlobData* blobData;
	PXCBlobConfiguration* blobConfiguration;
#endif


};



RealsenseEngine::RealsenseEngine(const char *calibFilename)//, Vector2i requested_imageSize_rgb, Vector2i requested_imageSize_d)
	: ImageSourceEngine(calibFilename)
{

	data = new PrivateData();

	
#ifndef SDK20	
	data->pp = PXCSenseManager::CreateInstance();

	/* Sets file recording or playback */
	data->cm = data->pp->QueryCaptureManager();


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
#else

	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
	data->pipe.start(cfg);



#endif


}
RealsenseEngine::~RealsenseEngine()
{
}
void RealsenseEngine::getImagesMF(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, MeshFusion * mfdata)
{
#ifndef SDK20

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
											 cv::Point p(plist[i].x, plist[i].y);
											 mfdata->pointlist.push_back(p);

											 //mfdata->pointlist[i].x = plist[i].x;
											 //mfdata->pointlist[i].y = plist[i].y;

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

#else


data->data = data->pipe.wait_for_frames();
rs2::frame depth = data->data.get_depth_frame(); // Find and colorize the depth data
rs2::frame color = data->data.get_color_frame();            // Find the color data

short *dep = rawDepthImage->GetData(MEMORYDEVICE_CPU);

short * dd = (short *)depth.get_data();
memcpy(dep, dd, rawDepthImage->dataSize * sizeof(short));

Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);

char* baseAddr = (char*)color.get_data();
for (int i = 0; i < rgbImage->noDims.x * rgbImage->noDims.y; i++) {
	Vector4u newPix; char *oldPix = &(((char*)(baseAddr))[i * 3]);
	newPix.x = oldPix[0]; newPix.y = oldPix[1]; newPix.z = oldPix[2]; newPix.w = 255;
	rgb[i] = newPix;
}




#endif


	return;
}
bool RealsenseEngine::hasMoreImages(void)
{
	return true;
}
Vector2i RealsenseEngine::getDepthImageSize(void)
{
	try {
	
	

#ifndef SDK20
		if (data->sts<PXC_STATUS_NO_ERROR)
			return Vector2i(0, 0);

		PXCSizeI32 s = data->cm->QueryImageSize(PXCCapture::STREAM_TYPE_DEPTH);
		if (&s != NULL && s.width >= 0 && s.height >= 0)
			return Vector2i(s.width, s.height);
		else
			return Vector2i(0, 0);
#else
		if (data->data == NULL)
		{
			data->data = data->pipe.wait_for_frames();

		}

		rs2::depth_frame depth = data->data.first(rs2_stream::RS2_STREAM_DEPTH);
		float h = depth.get_height();
		float w = depth.get_width();
		return Vector2i(w, h);


#endif
	}
	catch (std::exception &em)
	{
	}

	return Vector2i(0, 0);

	
}
Vector2i RealsenseEngine::getRGBImageSize(void)
{
	try {

	


#ifndef SDK20
		if (data->sts<PXC_STATUS_NO_ERROR)
			return Vector2i(0, 0);

	PXCSizeI32 s = data->cm->QueryImageSize(PXCCapture::STREAM_TYPE_COLOR);
	if (&s != NULL && s.width >= 0 && s.height >= 0)
		return Vector2i(s.width, s.height);
	else
		return Vector2i(0, 0);
#else
		if (data->data == NULL)
		{
			data->data = data->pipe.wait_for_frames();

		}

		rs2::video_frame depth = data->data.get_color_frame();
		float h = depth.get_height();
		float w = depth.get_width();
		return Vector2i(w, h);


#endif
	}
	catch (std::exception &em)
	{
	}

	return Vector2i(0, 0);

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
