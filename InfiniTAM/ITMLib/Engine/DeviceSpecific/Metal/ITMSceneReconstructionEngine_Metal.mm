// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#ifdef COMPILE_WITH_METAL

#import "MetalContext.h"

#include "../../../Objects/ITMRenderState_VH.h"

#include "ITMSceneReconstructionEngine_Metal.h"
#include "../../DeviceAgnostic/ITMSceneReconstructionEngine.h"

id<MTLFunction> f_integrateIntoScene_vh_device;
id<MTLComputePipelineState> p_integrateIntoScene_vh_device;

id<MTLFunction> f_buildAllocAndVisibleType_vh_device;
id<MTLComputePipelineState> p_buildAllocAndVisibleType_vh_device;

id<MTLBuffer> paramsBuffer_sceneReconstruction;

using namespace ITMLib::Engine;

template<class TVoxel>
ITMSceneReconstructionEngine_Metal<TVoxel,ITMVoxelBlockHash>::ITMSceneReconstructionEngine_Metal(void)
 : ITMSceneReconstructionEngine_CPU<TVoxel,ITMVoxelBlockHash>()
{
    NSError *errors;
    
    f_integrateIntoScene_vh_device = [[[MetalContext instance]library]newFunctionWithName:@"integrateIntoScene_vh_device"];
    p_integrateIntoScene_vh_device = [[[MetalContext instance]device]newComputePipelineStateWithFunction:f_integrateIntoScene_vh_device error:&errors];
    
    f_buildAllocAndVisibleType_vh_device = [[[MetalContext instance]library]newFunctionWithName:@"buildAllocAndVisibleType_vh_device"];
    p_buildAllocAndVisibleType_vh_device = [[[MetalContext instance]device]newComputePipelineStateWithFunction:f_buildAllocAndVisibleType_vh_device error:&errors];
    
    paramsBuffer_sceneReconstruction = BUFFEREMPTY(16384);
}

template<class TVoxel>
void ITMSceneReconstructionEngine_Metal<TVoxel,ITMVoxelBlockHash>::IntegrateIntoScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
                                                                                      const ITMTrackingState *trackingState, const ITMRenderState *renderState)
{
    id<MTLCommandBuffer> commandBuffer = [[[MetalContext instance]commandQueue]commandBuffer];
    id<MTLComputeCommandEncoder> commandEncoder = [commandBuffer computeCommandEncoder];
    
    ITMRenderState_VH *renderState_vh = (ITMRenderState_VH*)renderState;
    
    IntegrateIntoScene_VH_Params *params = (IntegrateIntoScene_VH_Params*)[paramsBuffer_sceneReconstruction contents];
    params->rgbImgSize = view->rgb->noDims;
    params->depthImgSize = view->depth->noDims;
    params->_voxelSize = scene->sceneParams->voxelSize;
    params->M_d = trackingState->pose_d->M;
    if (TVoxel::hasColorInformation) params->M_rgb = view->calib->trafo_rgb_to_depth.calib_inv * trackingState->pose_d->M;
    
    params->projParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;
    params->projParams_rgb = view->calib->intrinsics_rgb.projectionParamsSimple.all;
    
    params->mu = scene->sceneParams->mu; params->maxW = scene->sceneParams->maxW;
    
    [commandEncoder setComputePipelineState:p_integrateIntoScene_vh_device];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) scene->localVBA.GetVoxelBlocks_MB()      offset:0 atIndex:0];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) scene->index.GetEntries_MB()             offset:0 atIndex:1];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState_vh->GetLiveEntryIDs_MB()     offset:0 atIndex:2];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) view->rgb->GetMetalBuffer()              offset:0 atIndex:3];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) view->depth->GetMetalBuffer()            offset:0 atIndex:4];
    [commandEncoder setBuffer:paramsBuffer_sceneReconstruction                                  offset:0 atIndex:5];
    
    MTLSize blockSize = {SDF_BLOCK_SIZE, SDF_BLOCK_SIZE, SDF_BLOCK_SIZE};
    MTLSize gridSize = {(NSUInteger)renderState_vh->noLiveEntries, 1, 1};
    
    [commandEncoder dispatchThreadgroups:gridSize threadsPerThreadgroup:blockSize];
    [commandEncoder endEncoding];
    
    [commandBuffer commit];
    
    [commandBuffer waitUntilCompleted];
}

template<class TVoxel>
void ITMSceneReconstructionEngine_Metal<TVoxel,ITMVoxelBlockHash>::BuildAllocAndVisibleType(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
                                                                                            const ITMTrackingState *trackingState, const ITMRenderState *renderState)
{
    id<MTLCommandBuffer> commandBuffer = [[[MetalContext instance]commandQueue]commandBuffer];
    id<MTLComputeCommandEncoder> commandEncoder = [commandBuffer computeCommandEncoder];
    
    ITMRenderState_VH *renderState_vh = (ITMRenderState_VH*)renderState;

    Vector2i depthImgSize = view->depth->noDims;
    float voxelSize = scene->sceneParams->voxelSize;
    
    Matrix4f invM_d; trackingState->pose_d->M.inv(invM_d);
    Vector4f invProjParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;
    invProjParams_d.x = 1.0f / invProjParams_d.x;
    invProjParams_d.y = 1.0f / invProjParams_d.y;
    
    BuildAllocVisibleType_VH_Params *params = (BuildAllocVisibleType_VH_Params*)[paramsBuffer_sceneReconstruction contents];
    params->invM_d = invM_d;
    params->invProjParams_d = invProjParams_d;
    params->depthImgSize = depthImgSize;
    params->others.x = scene->sceneParams->mu;
    params->others.y = 1.0f / (voxelSize * SDF_BLOCK_SIZE);
    params->others.z = scene->sceneParams->viewFrustum_min;
    params->others.w = scene->sceneParams->viewFrustum_max;
    
    memset(this->entriesAllocType->GetData(MEMORYDEVICE_CPU), 0, scene->index.noVoxelBlocks);
    memset(renderState_vh->GetEntriesVisibleType(), 0, scene->index.noVoxelBlocks);
    memset(this->blockCoords->GetData(MEMORYDEVICE_CPU), 0, scene->index.noVoxelBlocks * sizeof(Vector4s));
    
    [commandEncoder setComputePipelineState:p_buildAllocAndVisibleType_vh_device];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) this->entriesAllocType->GetMetalBuffer()     offset:0 atIndex:0];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) renderState_vh->GetEntriesVisibleType_MB()   offset:0 atIndex:1];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) this->blockCoords->GetMetalBuffer()          offset:0 atIndex:2];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) scene->index.GetEntries_MB()                 offset:0 atIndex:3];
    [commandEncoder setBuffer:(__bridge id<MTLBuffer>) view->depth->GetMetalBuffer()                offset:0 atIndex:4];
    [commandEncoder setBuffer:paramsBuffer_sceneReconstruction                                      offset:0 atIndex:5];
    
    MTLSize blockSize = {8, 8, 1};
    MTLSize gridSize = {(NSUInteger)ceil((float)view->depth->noDims.x / (float)blockSize.width),
        (NSUInteger)ceil((float)view->depth->noDims.y / (float)blockSize.height), 1};
    
    [commandEncoder dispatchThreadgroups:gridSize threadsPerThreadgroup:blockSize];
    [commandEncoder endEncoding];
    
    [commandBuffer commit];
    
    [commandBuffer waitUntilCompleted];
}

template<class TVoxel>
void ITMSceneReconstructionEngine_Metal<TVoxel, ITMVoxelBlockHash>::AllocateSceneFromDepth(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
                                                                                           const ITMTrackingState *trackingState, const ITMRenderState *renderState)
{
    Vector2i depthImgSize = view->depth->noDims;
    float voxelSize = scene->sceneParams->voxelSize;
    
    Matrix4f M_d = trackingState->pose_d->M;
    Vector4f projParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;
    
    ITMRenderState_VH *renderState_vh = (ITMRenderState_VH*)renderState;
    
    float mu = scene->sceneParams->mu;
    
    float *depth = view->depth->GetData(MEMORYDEVICE_CPU);
    int *voxelAllocationList = scene->localVBA.GetAllocationList();
    int *excessAllocationList = scene->index.GetExcessAllocationList();
    ITMHashEntry *hashTable = scene->index.GetEntries();
    ITMHashCacheState *cacheStates = scene->useSwapping ? scene->globalCache->GetCacheStates(false) : 0;
    int *liveEntryIDs = renderState_vh->GetLiveEntryIDs();
    uchar *entriesVisibleType = renderState_vh->GetEntriesVisibleType();
    uchar *entriesAllocType = this->entriesAllocType->GetData(MEMORYDEVICE_CPU);
    Vector4s *blockCoords = this->blockCoords->GetData(MEMORYDEVICE_CPU);
    int noTotalEntries = scene->index.noVoxelBlocks;
    
    bool useSwapping = scene->useSwapping;
    
    float oneOverVoxelSize = 1.0f / (voxelSize * SDF_BLOCK_SIZE);
    
    int lastFreeVoxelBlockId = scene->localVBA.lastFreeBlockId;
    int lastFreeExcessListId = scene->index.lastFreeExcessListId;
    
    int hashIdxLive = 0;
    
    this->BuildAllocAndVisibleType(scene, view, trackingState, renderState);
    
    //allocate
    for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++)
    {
        int vbaIdx, exlIdx;
        
        switch (entriesAllocType[targetIdx])
        {
            case 1: //needs allocation, fits in the ordered list
                vbaIdx = lastFreeVoxelBlockId; lastFreeVoxelBlockId--;
                
                if (vbaIdx >= 0) //there is room in the voxel block array
                {
                    Vector4s pt_block_all = blockCoords[targetIdx];
                    
                    ITMHashEntry hashEntry;
                    
                    hashEntry.pos.x = pt_block_all.x; hashEntry.pos.y = pt_block_all.y; hashEntry.pos.z = pt_block_all.z;
                    hashEntry.ptr = voxelAllocationList[vbaIdx];
                    hashEntry.offset = 0;
                    
                    hashTable[targetIdx] = hashEntry;
                }
                
                break;
            case 2: //needs allocation in the excess list
                vbaIdx = lastFreeVoxelBlockId; lastFreeVoxelBlockId--;
                exlIdx = lastFreeExcessListId; lastFreeExcessListId--;
                
                if (vbaIdx >= 0 && exlIdx >= 0) //there is room in the voxel block array and excess list
                {
                    Vector4s pt_block_all = blockCoords[targetIdx];
                    
                    ITMHashEntry hashEntry;
                    
                    hashEntry.pos.x = pt_block_all.x; hashEntry.pos.y = pt_block_all.y; hashEntry.pos.z = pt_block_all.z;
                    hashEntry.ptr = voxelAllocationList[vbaIdx];
                    hashEntry.offset = 0;
                    
                    int exlOffset = excessAllocationList[exlIdx];
                    
                    hashTable[targetIdx].offset = exlOffset + 1; //connect to child
                    
                    hashTable[SDF_BUCKET_NUM * SDF_ENTRY_NUM_PER_BUCKET + exlOffset] = hashEntry; //add child to the excess list
                    
                    entriesVisibleType[SDF_BUCKET_NUM * SDF_ENTRY_NUM_PER_BUCKET + exlOffset] = 1; //make child visible
                }
                
                break;
        }
    }
    
    //build visible list
    for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++)
    {
        unsigned char hashVisibleType = entriesVisibleType[targetIdx];
        const ITMHashEntry &hashEntry = hashTable[targetIdx];
        
        if ((hashVisibleType == 0 && hashEntry.ptr >= 0) || hashVisibleType == 2)
        {
            bool isVisibleEnlarged = false;
            
            if (useSwapping)
            {
                checkBlockVisibility<true>(hashVisibleType, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d, voxelSize, depthImgSize);
                entriesVisibleType[targetIdx] = isVisibleEnlarged;
            }
            else checkBlockVisibility<false>(hashVisibleType, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d, voxelSize, depthImgSize);
        }
        
        if (useSwapping)
        {
            if (entriesVisibleType[targetIdx] > 0 && cacheStates[targetIdx].cacheFromHost != 2) cacheStates[targetIdx].cacheFromHost = 1;
        }
        
        if (hashVisibleType > 0)
        {	
            liveEntryIDs[hashIdxLive] = targetIdx;
            hashIdxLive++;
        }
    }
    
    //reallocate deletes ones from previous swap operation
    if (useSwapping)
    {
        for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++)
        {
            int vbaIdx;
            ITMHashEntry hashEntry = hashTable[targetIdx];
            
            if (entriesVisibleType[targetIdx] > 0 && hashEntry.ptr == -1) 
            {
                vbaIdx = lastFreeVoxelBlockId; lastFreeVoxelBlockId--;
                if (vbaIdx >= 0) hashTable[targetIdx].ptr = voxelAllocationList[vbaIdx];
            }
        }
    }
    
    renderState_vh->noLiveEntries = hashIdxLive;
    scene->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
    scene->index.lastFreeExcessListId = lastFreeExcessListId;
}

template class ITMLib::Engine::ITMSceneReconstructionEngine_Metal<ITMVoxel, ITMVoxelIndex>;

#endif