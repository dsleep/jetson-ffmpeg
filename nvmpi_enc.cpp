#include "nvmpi.h"
#include "NvVideoEncoder.h"

#if JETPACK_VER == 4
	#include "NvVideoConverter.h"
#elif JETPACK_VER >= 5
	#include "nvbuf_utils.h"
	#include "NvUtils.h"
	#include "NvBufSurface.h"
#endif

#include <vector>
#include <iostream>
#include <thread>
#include <unistd.h>
#include <queue>
#include <string>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <functional>

#define CHUNK_SIZE 2*1024*1024
#define MAX_BUFFERS 32
#define TEST_ERROR(condition, message, errorCode)    \
	if (condition)                               \
{                                                    \
	std::cout<< message;                         \
}


using namespace std;

#if JETPACK_VER == 4
struct NvImagePlaneConverter
{
	std::condition_variable cv;
	std::mutex cv_m;

	int bufIndex = 0;
	std::unique_ptr<NvVideoConverter> yuvConverter;
	NvBuffer *lastBuf = nullptr;

	static bool _converterCapturePlaneDqCallback(struct v4l2_buffer *v4l2_buf, NvBuffer *buffer, NvBuffer *shared_buffer, void *arg)
	{
		return static_cast<NvImagePlaneConverter*>(arg)->converterCapturePlaneDqCallback(v4l2_buf, buffer, shared_buffer);
	}

	bool converterCapturePlaneDqCallback(struct v4l2_buffer *v4l2_buf, NvBuffer *buffer, NvBuffer *shared_buffer)
	{
		std::cout << "NvImagePlaneConverter::converterCapturePlaneDqCallback" << endl;	
			
		{
			std::unique_lock<std::mutex> lk(cv_m);			
			if (buffer)
			{			
				std::cout << " - has buf" << endl;
				lastBuf = buffer;
			}
			else
			{
				std::cout << " - no buf" << endl;
			}
		}
				
		cv.notify_one();
		
		if (buffer == nullptr || buffer->planes[0].bytesused == 0)
		{								
			std::cout << "converterCapturePlaneDqCallback EOS" << endl;
			return false;
		}
		
		if (yuvConverter->capture_plane.qBuffer(*v4l2_buf, NULL) < 0)
		{
			std::cout << "Error while Qing buffer at capture plane" << endl;
			return false;
		}
						
		return true;
	}
	
	void ShutDown()
	{
		std::cout << "NvImagePlaneConverter::ShutDown" << endl;			
		
		if(yuvConverter)
		{
			PushRGBFrame(nullptr, 0, nullptr);
			yuvConverter->output_plane.setStreamStatus(false);
			yuvConverter->capture_plane.setStreamStatus(false);
			yuvConverter->capture_plane.stopDQThread();
		}
	}

	void Initialize(int32_t InFrameWidth, int32_t InFrameHeight)
	{
		std::cout << "NvImagePlaneConverter::Initialize: " << InFrameWidth << " " << InFrameHeight << endl;	
		
		bufIndex = 0;

		yuvConverter = std::unique_ptr<NvVideoConverter>(NvVideoConverter::createVideoConverter("conv0"));

		auto ret = yuvConverter->setOutputPlaneFormat(V4L2_PIX_FMT_ABGR32, InFrameWidth, InFrameHeight, V4L2_NV_BUFFER_LAYOUT_PITCH);
		ret = yuvConverter->setCapturePlaneFormat(V4L2_PIX_FMT_YUV420M, InFrameWidth, InFrameHeight, V4L2_NV_BUFFER_LAYOUT_PITCH);

		ret = yuvConverter->output_plane.setupPlane(V4L2_MEMORY_USERPTR, 1, false, true);
		ret = yuvConverter->capture_plane.setupPlane(V4L2_MEMORY_MMAP, 1, true, false);
		
		yuvConverter->output_plane.setStreamStatus(true);
		yuvConverter->capture_plane.setStreamStatus(true);
		
		yuvConverter->capture_plane.setDQThreadCallback(_converterCapturePlaneDqCallback);
		yuvConverter->capture_plane.startDQThread(this);

		std::cout << " - buffer count: " << yuvConverter->capture_plane.getNumBuffers() << endl;	
		for (uint32_t i = 0; i < yuvConverter->capture_plane.getNumBuffers(); i++)
		{
			struct v4l2_buffer v4l2_buf;
			struct v4l2_plane planes[MAX_PLANES];

			memset(&v4l2_buf, 0, sizeof(v4l2_buf));
			memset(planes, 0, MAX_PLANES * sizeof(struct v4l2_plane));

			v4l2_buf.index = i;
			v4l2_buf.m.planes = planes;

			yuvConverter->capture_plane.qBuffer(v4l2_buf, NULL);
		}
	}

	bool PushRGBFrame(const void *InData, int32_t DataSize, const std::function<void(NvBuffer*)> &InCompl)
	{				
		std::cout << "PushRGBFrame: " << DataSize << endl;	
	
		struct v4l2_buffer v4l2_buf;
		struct v4l2_plane planes[MAX_PLANES];		
		NvBuffer *nvBuffer = nullptr;

		memset(&v4l2_buf, 0, sizeof(v4l2_buf));
		memset(planes, 0, MAX_PLANES * sizeof(struct v4l2_plane));

		v4l2_buf.m.planes = planes;

		if (yuvConverter->isInError())
			return false;

		int ret = 0;
		
		std::cout << " - bufIndex: " << bufIndex << endl;	
		if (bufIndex < yuvConverter->output_plane.getNumBuffers())
		{
			nvBuffer = yuvConverter->output_plane.getNthBuffer(bufIndex);
			v4l2_buf.index = bufIndex;
			bufIndex++;
		}
		else
		{
			ret = yuvConverter->output_plane.dqBuffer(v4l2_buf, &nvBuffer, NULL, -1);
			if (ret < 0) {
				cout << "Error DQing buffer at output plane" << std::endl;
				return false;
			}
		}

		memcpy(nvBuffer->planes[0].data, InData, DataSize);		
		nvBuffer->planes[0].bytesused = DataSize;
				
					
		{
			using namespace std::chrono_literals;
			std::unique_lock<std::mutex> lk(cv_m);
			
			cout << " - queueing... " << std::endl;
			yuvConverter->output_plane.qBuffer(v4l2_buf, nullptr);
			
			if(InCompl)
			{
				if (cv.wait_for(lk, 10s, [&] {return lastBuf; }))
				{
					std::cout << " - run InCompl passed";	
					InCompl(lastBuf);
				}
				else
				{
					std::cout << " - run InCompl failed";	
					InCompl(nullptr);
				}
			}
			lastBuf = nullptr;
		}
		
		return true;
	}
};
#elif JETPACK_VER >= 5
struct NvBufferConverterData
{
	int in_dmabuf_fd = -1;
    int out_dmabuf_fd = -1;
    NvBufSurf::NvCommonAllocateParams input_params;
    NvBufSurf::NvCommonAllocateParams output_params;
    NvBufSurf::NvCommonTransformParams transform_params;
    vector<int> src_fmt_bytes_per_pixel;
    vector<int> dest_fmt_bytes_per_pixel;
    NvBufSurfTransformSyncObj_t syncobj;

	std::vector<uint8_t> planeConversionData[3];
	

	NvBufferConverterData() {}

	~NvBufferConverterData()
	{
		if (in_dmabuf_fd != -1)
		{
			NvBufSurf::NvDestroy(in_dmabuf_fd);
			in_dmabuf_fd = -1;
		}

		if (out_dmabuf_fd != -1)
		{
			NvBufSurf::NvDestroy(out_dmabuf_fd);
			out_dmabuf_fd = -1;
		}
	}
};
#endif

static const char hex_characters[] = {'0', '1', '2', '3', '4', '5', '6', 
		'7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

std::string hex_string(int length)
{
	std::string oString;
	oString.reserve(length);
	for (int32_t i = 0; i < length; i++)
	{
		oString += hex_characters[rand() % 16];
	}
	return oString;
}

struct nvmpictx
{
	std::string GUID;
		
	std::unique_ptr<NvVideoEncoder> enc;
	int encIndex;

#if JETPACK_VER == 4
	std::unique_ptr<NvImagePlaneConverter> ImgPlaneConverter;
#elif JETPACK_VER >= 5
	std::unique_ptr<NvBufferConverterData> nvBufConverter;
#endif
	
	std::queue<int> * packet_pools;
	uint32_t width;
	uint32_t height;
	uint32_t profile;
	bool enableLossless;
	bool enableImageConverter;
	uint32_t bitrate;
	uint32_t peak_bitrate;
	uint32_t raw_pixfmt;
	uint32_t encoder_pixfmt;
	enum v4l2_mpeg_video_bitrate_mode ratecontrol;
	enum v4l2_mpeg_video_h264_level level;
	enum v4l2_enc_hw_preset_type hw_preset_type;
	uint32_t iframe_interval;
	uint32_t idr_interval;
	uint32_t fps_n;
	uint32_t fps_d;
	bool enable_extended_colorformat;
	uint32_t qmax;
	uint32_t qmin;
	uint32_t num_b_frames;
	uint32_t num_reference_frames;
	bool insert_sps_pps_at_idr;

	uint32_t packets_buf_size;
	uint32_t packets_num;
	unsigned char * packets[MAX_BUFFERS];
	uint32_t packets_size[MAX_BUFFERS];
	bool packets_keyflag[MAX_BUFFERS];
	uint64_t timestamp[MAX_BUFFERS];
	int buf_index;
};

#if JETPACK_VER >= 5
/**
 * This function returns vector contians bytes per pixel info
 * of each plane in sequence.
**/
static int
fill_bytes_per_pixel(NvBufSurfaceColorFormat pixel_format, vector<int> *bytes_per_pixel_fmt)
{
    switch (pixel_format)
    {
        case NVBUF_COLOR_FORMAT_NV12:
        case NVBUF_COLOR_FORMAT_NV12_ER:
        case NVBUF_COLOR_FORMAT_NV21:
        case NVBUF_COLOR_FORMAT_NV21_ER:
        case NVBUF_COLOR_FORMAT_NV12_709:
        case NVBUF_COLOR_FORMAT_NV12_709_ER:
        case NVBUF_COLOR_FORMAT_NV12_2020:
        case NVBUF_COLOR_FORMAT_NV16:
        case NVBUF_COLOR_FORMAT_NV24:
        case NVBUF_COLOR_FORMAT_NV16_ER:
        case NVBUF_COLOR_FORMAT_NV24_ER:
        case NVBUF_COLOR_FORMAT_NV16_709:
        case NVBUF_COLOR_FORMAT_NV24_709:
        case NVBUF_COLOR_FORMAT_NV16_709_ER:
        case NVBUF_COLOR_FORMAT_NV24_709_ER:
        {
            bytes_per_pixel_fmt->push_back(1);
            bytes_per_pixel_fmt->push_back(2);
            break;
        }
        case NVBUF_COLOR_FORMAT_NV12_10LE:
        case NVBUF_COLOR_FORMAT_NV12_10LE_709:
        case NVBUF_COLOR_FORMAT_NV12_10LE_709_ER:
        case NVBUF_COLOR_FORMAT_NV12_10LE_2020:
        case NVBUF_COLOR_FORMAT_NV21_10LE:
        case NVBUF_COLOR_FORMAT_NV12_12LE:
        case NVBUF_COLOR_FORMAT_NV12_12LE_2020:
        case NVBUF_COLOR_FORMAT_NV21_12LE:
        case NVBUF_COLOR_FORMAT_NV16_10LE:
        case NVBUF_COLOR_FORMAT_NV24_10LE_709:
        case NVBUF_COLOR_FORMAT_NV24_10LE_709_ER:
        case NVBUF_COLOR_FORMAT_NV24_10LE_2020:
        case NVBUF_COLOR_FORMAT_NV24_12LE_2020:
        {
            bytes_per_pixel_fmt->push_back(2);
            bytes_per_pixel_fmt->push_back(4);
            break;
        }
        case NVBUF_COLOR_FORMAT_ABGR:
		case NVBUF_COLOR_FORMAT_BGRA:
		case NVBUF_COLOR_FORMAT_BGRx:        
        case NVBUF_COLOR_FORMAT_ARGB:
		case NVBUF_COLOR_FORMAT_RGBx:
        {
            bytes_per_pixel_fmt->push_back(4);
            break;
        }
        case NVBUF_COLOR_FORMAT_YUV420:
        case NVBUF_COLOR_FORMAT_YUV420_ER:
        case NVBUF_COLOR_FORMAT_YUV420_709:
        case NVBUF_COLOR_FORMAT_YUV420_709_ER:
        case NVBUF_COLOR_FORMAT_YUV420_2020:
        case NVBUF_COLOR_FORMAT_YUV444:
        {
            bytes_per_pixel_fmt->push_back(1);
            bytes_per_pixel_fmt->push_back(1);
            bytes_per_pixel_fmt->push_back(1);
            break;
        }
        case NVBUF_COLOR_FORMAT_UYVY:
        case NVBUF_COLOR_FORMAT_UYVY_ER:
        case NVBUF_COLOR_FORMAT_VYUY:
        case NVBUF_COLOR_FORMAT_VYUY_ER:
        case NVBUF_COLOR_FORMAT_YUYV:
        case NVBUF_COLOR_FORMAT_YUYV_ER:
        case NVBUF_COLOR_FORMAT_YVYU:
        case NVBUF_COLOR_FORMAT_YVYU_ER:
        {
            bytes_per_pixel_fmt->push_back(2);
            break;
        }
        case NVBUF_COLOR_FORMAT_GRAY8:
        {
            bytes_per_pixel_fmt->push_back(1);
            break;
        }
        default:
            return -1;
    }
    return 0;
}
#endif

static bool encoder_capture_plane_dq_callback(struct v4l2_buffer *v4l2_buf, 
	NvBuffer * buffer, 
	NvBuffer * shared_buffer, 
	void *arg)
{

	nvmpictx *ctx = (nvmpictx *) arg;
	NvVideoEncoder *enc = ctx->enc.get();
	//uint32_t frame_num = ctx->enc->capture_plane.getTotalDequeuedBuffers() - 1;

	if (v4l2_buf == NULL)
	{
		cout << "Error while dequeing buffer from output plane" << endl;
		enc->capture_plane.setStreamStatus(false);
		return false;
	}

	if (buffer->planes[0].bytesused == 0)
	{
		cout << "Got 0 size buffer in capture EOS \n";
		enc->capture_plane.setStreamStatus(false);
		return false;
	}

	if(ctx->packets_buf_size < buffer->planes[0].bytesused)
	{
		ctx->packets_buf_size=buffer->planes[0].bytesused;
		for(int index=0;index< ctx->packets_num;index++)
		{
			delete[] ctx->packets[index];
			ctx->packets[index]=new unsigned char[ctx->packets_buf_size];	
		}
	}

	ctx->packets_size[ctx->buf_index]=buffer->planes[0].bytesused;
	memcpy(ctx->packets[ctx->buf_index],buffer->planes[0].data,buffer->planes[0].bytesused);
	ctx->timestamp[ctx->buf_index] = (v4l2_buf->timestamp.tv_usec % 1000000) + (v4l2_buf->timestamp.tv_sec * 1000000UL);
	ctx->packet_pools->push(ctx->buf_index);

	v4l2_ctrl_videoenc_outputbuf_metadata enc_metadata;
	ctx->enc->getMetadata(v4l2_buf->index, enc_metadata);
	if(enc_metadata.KeyFrame)
	{
		ctx->packets_keyflag[ctx->buf_index]=true;
	}
	else
	{
		ctx->packets_keyflag[ctx->buf_index]=false;
	}

	ctx->buf_index=(ctx->buf_index+1)%ctx->packets_num;	

	if (ctx->enc->capture_plane.qBuffer(*v4l2_buf, NULL) < 0)
	{
		ERROR_MSG("Error while Qing buffer at capture plane");
		return false;
	}

	return true;
}

nvmpictx* nvmpi_create_encoder(nvCodingType codingType,nvEncParam * param){

	int ret;
	log_level = LOG_LEVEL_ERROR;
	nvmpictx *ctx=new nvmpictx;

	srand(time(0));

	ctx->GUID = hex_string(4);
	ctx->encIndex=0;

	std::cout << "******** NEW ENCODER *********" << endl;
	std::cout << "nvmpi_create_encoder: " << ctx->GUID << endl;
	
	ctx->width=param->width;
	ctx->height=param->height;
	ctx->enableLossless=false;
	ctx->enableImageConverter=param->isRawRGBA;
	
	ctx->bitrate=param->bitrate;
	ctx->ratecontrol = V4L2_MPEG_VIDEO_BITRATE_MODE_CBR;	
	ctx->idr_interval = param->idr_interval;
	ctx->fps_n = param->fps_n;
	ctx->fps_d = param->fps_d;
	ctx->iframe_interval = param->iframe_interval;
	ctx->packet_pools=new std::queue<int>;
	ctx->buf_index=0;
	ctx->enable_extended_colorformat=false;
	ctx->packets_num=param->capture_num;
	ctx->qmax=param->qmax;
	ctx->qmin=param->qmin;
	ctx->num_b_frames=param->max_b_frames;
	ctx->num_reference_frames=param->refs;
	ctx->insert_sps_pps_at_idr=(param->insert_spspps_idr==1)?true:false;

	switch(param->profile){
		case 77://FF_PROFILE_H264_MAIN
			ctx->profile=V4L2_MPEG_VIDEO_H264_PROFILE_MAIN;
			break;
		case 66://FF_PROFILE_H264_BASELINE
			ctx->profile=V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE;
			break;
		case 100://FF_PROFILE_H264_HIGH
			ctx->profile=V4L2_MPEG_VIDEO_H264_PROFILE_HIGH;
			break;

		default:
			ctx->profile=V4L2_MPEG_VIDEO_H264_PROFILE_MAIN;
			break;

	}

	switch(param->level){
		case 10:
			ctx->level=V4L2_MPEG_VIDEO_H264_LEVEL_1_0;
			break;
		case 11:
			ctx->level=V4L2_MPEG_VIDEO_H264_LEVEL_1_1;
			break;
		case 12:
			ctx->level=V4L2_MPEG_VIDEO_H264_LEVEL_1_2;
			break;
		case 13:
			ctx->level=V4L2_MPEG_VIDEO_H264_LEVEL_1_3;
			break;
		case 20:
			ctx->level=V4L2_MPEG_VIDEO_H264_LEVEL_2_0;
			break;
		case 21:
			ctx->level=V4L2_MPEG_VIDEO_H264_LEVEL_2_1;
			break;
		case 22:
			ctx->level=V4L2_MPEG_VIDEO_H264_LEVEL_2_2;
			break;
		case 30:
			ctx->level=V4L2_MPEG_VIDEO_H264_LEVEL_3_0;
			break;
		case 31:
			ctx->level=V4L2_MPEG_VIDEO_H264_LEVEL_3_1;
			break;
		case 32:
			ctx->level=V4L2_MPEG_VIDEO_H264_LEVEL_3_2;
			break;
		case 40:
			ctx->level=V4L2_MPEG_VIDEO_H264_LEVEL_4_0;
			break;
		case 41:
			ctx->level=V4L2_MPEG_VIDEO_H264_LEVEL_4_1;
			break;
		case 42:
			ctx->level=V4L2_MPEG_VIDEO_H264_LEVEL_4_2;
			break;
		case 50:
			ctx->level=V4L2_MPEG_VIDEO_H264_LEVEL_5_0;
			break;
		case 51:
			ctx->level=V4L2_MPEG_VIDEO_H264_LEVEL_5_1;
			break;
		default:
			ctx->level=V4L2_MPEG_VIDEO_H264_LEVEL_5_1;
			break;	
	}

	switch(param->hw_preset_type){
		case 1:
			ctx->hw_preset_type = V4L2_ENC_HW_PRESET_ULTRAFAST;
			break;
		case 2:
			ctx->hw_preset_type = V4L2_ENC_HW_PRESET_FAST;
			break;
		case 3:
			ctx->hw_preset_type = V4L2_ENC_HW_PRESET_MEDIUM;
			break;
		case 4:
			ctx->hw_preset_type = V4L2_ENC_HW_PRESET_SLOW;
			break;
		default:
			ctx->hw_preset_type = V4L2_ENC_HW_PRESET_MEDIUM;
			break;

	}



	if(param->enableLossless)
		ctx->enableLossless=true;

	if(param->mode_vbr)
		ctx->ratecontrol=V4L2_MPEG_VIDEO_BITRATE_MODE_VBR;

	ctx->packets_buf_size=CHUNK_SIZE;

	for(int index=0;index<MAX_BUFFERS;index++)
		ctx->packets[index]=new unsigned char[ctx->packets_buf_size];

	if(codingType==NV_VIDEO_CodingH264){
		ctx->encoder_pixfmt=V4L2_PIX_FMT_H264;
	}else if(codingType==NV_VIDEO_CodingHEVC){
		ctx->encoder_pixfmt=V4L2_PIX_FMT_H265;
	}
	ctx->enc.reset( NvVideoEncoder::createVideoEncoder("enc0") );
	TEST_ERROR(!ctx->enc, "Could not create encoder",ret);

	ret = ctx->enc->setCapturePlaneFormat(ctx->encoder_pixfmt, ctx->width,ctx->height, CHUNK_SIZE);

	TEST_ERROR(ret < 0, "Could not set output plane format", ret);

	switch (ctx->profile)
	{
		case V4L2_MPEG_VIDEO_H265_PROFILE_MAIN10:
			ctx->raw_pixfmt = V4L2_PIX_FMT_P010M;
			break;
		case V4L2_MPEG_VIDEO_H265_PROFILE_MAIN:
		default:
			ctx->raw_pixfmt = V4L2_PIX_FMT_YUV420M;
	}

	if (ctx->enableLossless && codingType == NV_VIDEO_CodingH264)
	{
		ctx->profile = V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_PREDICTIVE;
		ret = ctx->enc->setOutputPlaneFormat(V4L2_PIX_FMT_YUV444M, ctx->width,ctx->height);
	}
	else{
		ret = ctx->enc->setOutputPlaneFormat(ctx->raw_pixfmt, ctx->width,ctx->height);
	}

	TEST_ERROR(ret < 0, "Could not set output plane format", ret);

	ret = ctx->enc->setBitrate(ctx->bitrate);
	TEST_ERROR(ret < 0, "Could not set encoder bitrate", ret);

	ret=ctx->enc->setHWPresetType(ctx->hw_preset_type);
	TEST_ERROR(ret < 0, "Could not set encoder HW Preset Type", ret);

	if(ctx->num_reference_frames){
		ret = ctx->enc->setNumReferenceFrames(ctx->num_reference_frames);
		TEST_ERROR(ret < 0, "Could not set num reference frames", ret);
	}

	if(ctx->num_b_frames != (uint32_t) -1 && codingType == NV_VIDEO_CodingH264 ){
		ret = ctx->enc->setNumBFrames(ctx->num_b_frames);
		TEST_ERROR(ret < 0, "Could not set number of B Frames", ret);
	}


	if (codingType == NV_VIDEO_CodingH264 || codingType == NV_VIDEO_CodingHEVC)
	{
		ret = ctx->enc->setProfile(ctx->profile);
		TEST_ERROR(ret < 0, "Could not set encoder profile", ret);
	}

	if( codingType== NV_VIDEO_CodingH264){
		ret = ctx->enc->setLevel(ctx->level);
		TEST_ERROR(ret < 0, "Could not set encoder level", ret);
	}


	if (ctx->enableLossless){
		ret = ctx->enc->setConstantQp(0);
		TEST_ERROR(ret < 0, "Could not set encoder constant qp=0", ret);

	}else{

		ret = ctx->enc->setRateControlMode(ctx->ratecontrol);
		TEST_ERROR(ret < 0, "Could not set encoder rate control mode", ret);

		if (ctx->ratecontrol == V4L2_MPEG_VIDEO_BITRATE_MODE_VBR){

			uint32_t peak_bitrate;
			if (ctx->peak_bitrate < ctx->bitrate)
				peak_bitrate = 1.2f * ctx->bitrate;
			else
				peak_bitrate = ctx->peak_bitrate;
			ret = ctx->enc->setPeakBitrate(peak_bitrate);
			TEST_ERROR(ret < 0, "Could not set encoder peak bitrate", ret);
		}
	}

	ret = ctx->enc->setIDRInterval(ctx->idr_interval);
	TEST_ERROR(ret < 0, "Could not set encoder IDR interval", ret);

	if(ctx->qmax>0 ||ctx->qmin >0){
		ctx->enc->setQpRange(ctx->qmin, ctx->qmax, ctx->qmin,ctx->qmax, ctx->qmin, ctx->qmax);	
	}
	ret = ctx->enc->setIFrameInterval(ctx->iframe_interval);
	TEST_ERROR(ret < 0, "Could not set encoder I-Frame interval", ret);
	
	if(ctx->insert_sps_pps_at_idr){
		ret = ctx->enc->setInsertSpsPpsAtIdrEnabled(true);
		TEST_ERROR(ret < 0, "Could not set insertSPSPPSAtIDR", ret);
	}

	ret = ctx->enc->setFrameRate(ctx->fps_n, ctx->fps_d);
	TEST_ERROR(ret < 0, "Could not set framerate", ret);

	ret = ctx->enc->output_plane.setupPlane(V4L2_MEMORY_USERPTR, ctx->packets_num, false, true);
	TEST_ERROR(ret < 0, "Could not setup output plane", ret);

	ret = ctx->enc->capture_plane.setupPlane(V4L2_MEMORY_MMAP, ctx->packets_num, true, false);
	TEST_ERROR(ret < 0, "Could not setup capture plane", ret);

	ret = ctx->enc->subscribeEvent(V4L2_EVENT_EOS,0,0);
	TEST_ERROR(ret < 0, "Could not subscribe EOS event", ret);

	ret = ctx->enc->output_plane.setStreamStatus(true);
	TEST_ERROR(ret < 0, "Error in output plane streamon", ret);

	ret = ctx->enc->capture_plane.setStreamStatus(true);
	TEST_ERROR(ret < 0, "Error in capture plane streamon", ret);


	ctx->enc->capture_plane.setDQThreadCallback(encoder_capture_plane_dq_callback);

	ctx->enc->capture_plane.startDQThread(ctx);

	// Enqueue all the empty capture plane buffers
	for (uint32_t i = 0; i < ctx->enc->capture_plane.getNumBuffers(); i++)
	{
		struct v4l2_buffer v4l2_buf;
		struct v4l2_plane planes[MAX_PLANES];
		memset(&v4l2_buf, 0, sizeof(v4l2_buf));
		memset(planes, 0, MAX_PLANES * sizeof(struct v4l2_plane));

		v4l2_buf.index = i;
		v4l2_buf.m.planes = planes;

		ret = ctx->enc->capture_plane.qBuffer(v4l2_buf, NULL);
		TEST_ERROR(ret < 0, "Error while queueing buffer at capture plane", ret);
	}

	// if using img converter
#if JETPACK_VER == 4
	if (ctx->enableImageConverter)
	{
		ctx->ImgPlaneConverter = std::make_unique<NvImagePlaneConverter>();
		ctx->ImgPlaneConverter->Initialize(ctx->width, ctx->height);
	}
#elif JETPACK_VER >= 5
	if(ctx->enableImageConverter)
	{
		cout << "nvmpi - enabling image converter" << std::endl;
			
		ctx->nvBufConverter = std::make_unique<NvBufferConverterData>();

		ctx->nvBufConverter->input_params.width = ctx->width;
		ctx->nvBufConverter->input_params.height = ctx->height;
		ctx->nvBufConverter->input_params.layout = NVBUF_LAYOUT_PITCH;
		ctx->nvBufConverter->input_params.memType = NVBUF_MEM_SURFACE_ARRAY;
		ctx->nvBufConverter->input_params.colorFormat = NVBUF_COLOR_FORMAT_BGRx;
		ctx->nvBufConverter->input_params.memtag = NvBufSurfaceTag_VIDEO_CONVERT;

		ctx->nvBufConverter->output_params.width = ctx->width;
		ctx->nvBufConverter->output_params.height = ctx->height;
		ctx->nvBufConverter->output_params.layout = NVBUF_LAYOUT_PITCH;
		ctx->nvBufConverter->output_params.memType = NVBUF_MEM_SURFACE_ARRAY;
		ctx->nvBufConverter->output_params.colorFormat = NVBUF_COLOR_FORMAT_YUV420;
		ctx->nvBufConverter->output_params.memtag = NvBufSurfaceTag_VIDEO_CONVERT;

		/* Create the HW Buffer. It is exported as
		** an FD by the hardware.
		*/
		ctx->nvBufConverter->in_dmabuf_fd = -1;
		ret = NvBufSurf::NvAllocate(&ctx->nvBufConverter->input_params, 1, &ctx->nvBufConverter->in_dmabuf_fd);
		if (ret)
		{
			cerr << "Error in creating the input buffer." << endl;			
		}

		ctx->nvBufConverter->out_dmabuf_fd = -1;
		ret = NvBufSurf::NvAllocate(&ctx->nvBufConverter->output_params, 1, &ctx->nvBufConverter->out_dmabuf_fd);
		if (ret)
		{
			cerr << "Error in creating the output buffer." << endl;
		}

		/* Store th bpp required for each color
		 ** format to read/write properly to raw
		 ** buffers.
		 */
		ret = fill_bytes_per_pixel(ctx->nvBufConverter->input_params.colorFormat, &ctx->nvBufConverter->src_fmt_bytes_per_pixel);
		if (ret)
		{
			cerr << "Error figure out bytes per pixel for source format." << endl;		
		}
		else
		{
			cout << "Planes for src: " << ctx->nvBufConverter->src_fmt_bytes_per_pixel.size() << endl;
		}
		ret = fill_bytes_per_pixel(ctx->nvBufConverter->output_params.colorFormat, &ctx->nvBufConverter->dest_fmt_bytes_per_pixel);
		if (ret)
		{
			cerr << "Error figure out bytes per pixel for destination format." << endl;		
		}
		else
		{
			cout << "Planes for dst: " << ctx->nvBufConverter->dest_fmt_bytes_per_pixel.size() << endl;
		}
		/* @transform_flag defines the flags for
		** enabling the valid transforms.
		** All the valid parameters are present in
		** the nvbuf_utils header.
		*/
		memset(&ctx->nvBufConverter->transform_params, 0, sizeof(ctx->nvBufConverter->transform_params));
		ctx->nvBufConverter->transform_params.src_top = 0;
		ctx->nvBufConverter->transform_params.src_left = 0;
		ctx->nvBufConverter->transform_params.src_width = ctx->width;
		ctx->nvBufConverter->transform_params.src_height = ctx->height;
		ctx->nvBufConverter->transform_params.dst_top = 0;
		ctx->nvBufConverter->transform_params.dst_left = 0;
		ctx->nvBufConverter->transform_params.dst_width = ctx->width;
		ctx->nvBufConverter->transform_params.dst_height = ctx->height;
		ctx->nvBufConverter->transform_params.flag = (NvBufSurfTransform_Transform_Flag)(NVBUFSURF_TRANSFORM_FILTER | NVBUFSURF_TRANSFORM_FLIP);
		
	    ctx->nvBufConverter->transform_params.flip = NvBufSurfTransform_None;
	    ctx->nvBufConverter->transform_params.filter = NvBufSurfTransformInter_Nearest;

		cout << "nvmpi - post enable" << std::endl;
	}
#endif
	return ctx;
}

	
int nvmpi_video_put_frame(nvmpictx* ctx,
	unsigned long payload_size[3],
	unsigned char *payload[3],
	const time_t &timestamp,
	NvBuffer *buffer
	)
{		
	cout << "nvmpi_video_put_frame: " << payload_size[0] << std::endl;

	int ret = -1;

	struct v4l2_buffer v4l2_buf;
	struct v4l2_plane planes[MAX_PLANES];
	NvBuffer *nvBuffer = nullptr;

	memset(&v4l2_buf, 0, sizeof(v4l2_buf));
	memset(planes, 0, sizeof(planes));

	v4l2_buf.m.planes = planes;

	if(ctx->enc->isInError())
		return -1;

	if(ctx->encIndex < ctx->enc->output_plane.getNumBuffers())
	{
		nvBuffer=ctx->enc->output_plane.getNthBuffer(ctx->encIndex);
		v4l2_buf.index = ctx->encIndex;
		ctx->encIndex++;
	}
	else
	{
		ret = ctx->enc->output_plane.dqBuffer(v4l2_buf, &nvBuffer, NULL, -1);
		if (ret < 0) {
			cout << "Error DQing buffer at output plane" << std::endl;
			return false;
		}
	}

	v4l2_buf.flags |= V4L2_BUF_FLAG_TIMESTAMP_COPY;
	v4l2_buf.timestamp.tv_usec = timestamp % 1000000;
	v4l2_buf.timestamp.tv_sec = timestamp / 1000000;
	
	if(buffer)
	{		
		ret = ctx->enc->output_plane.qBuffer(v4l2_buf, buffer);
	}
	else
	{		
		if(payload_size[0])memcpy(nvBuffer->planes[0].data,payload[0],payload_size[0]);
		if(payload_size[1])memcpy(nvBuffer->planes[1].data,payload[1],payload_size[1]);
		if(payload_size[2])memcpy(nvBuffer->planes[2].data,payload[2],payload_size[2]);
		nvBuffer->planes[0].bytesused=payload_size[0];
		nvBuffer->planes[1].bytesused=payload_size[1];
		nvBuffer->planes[2].bytesused=payload_size[2];
		ret = ctx->enc->output_plane.qBuffer(v4l2_buf, NULL);		
	}
	
	TEST_ERROR(ret < 0, "Error while queueing buffer at output plane", ret);

	return 0;
}


#if JETPACK_VER == 4

int nvmpi_converter_put_frame(nvmpictx* ctx, nvFrame* frame)
{	
	int ret = 0;
	
	std::cout << "nvmpi_converter_put_frame: " << ctx->GUID << endl;	
	ctx->ImgPlaneConverter->PushRGBFrame(frame->payload[0], frame->payload_size[0], [&](NvBuffer* InBuffer)
	{
		std::cout << "nvmpi_converter_put_frame: IN COMPLETION - " << ctx->GUID << endl;
		
		unsigned long payload_size[3] = { InBuffer->planes[0].bytesused, InBuffer->planes[1].bytesused, InBuffer->planes[2].bytesused };
		unsigned char *payload[3] = { InBuffer->planes[0].data, InBuffer->planes[1].data, InBuffer->planes[2].data };

		std::cout << " - payloads: " << payload_size[0] << " : " << payload_size[1] << " : " << payload_size[2] << endl;

		//auto copySize = nvbuf_surf_dst->surfaceList[0].planeParams.pitch[plane] * nvbuf_surf_dst->surfaceList[0].planeParams.height[plane];
	

		ret = nvmpi_video_put_frame(ctx, payload_size, payload, frame->timestamp, InBuffer);
	});
	return ret;
}

#elif JETPACK_VER >= 5

int nvmpi_converter_put_frame(nvmpictx* ctx,nvFrame* frame)
{		
	NvBufSurface *nvbuf_surf_src = 0;
    NvBufSurface *nvbuf_surf_dst = 0;
	
	unsigned long payload_size[3] = { 0 };
	unsigned char *payload[3] = { 0 };

	//std::cout << "nvmpi_converter_put_frame: " << ctx->GUID << endl;	

	int ret = 0;

	//std::cout << "nvmpi_converter_put_frame: " << frame->payload_size[0] << endl;	

    ret = NvBufSurfaceFromFd(ctx->nvBufConverter->in_dmabuf_fd, (void**)(&nvbuf_surf_src));
	if (ret) cerr << "Error in NvBufSurfaceFromFd src." << endl;		
	ret = NvBufSurfaceFromFd(ctx->nvBufConverter->out_dmabuf_fd, (void**)(&nvbuf_surf_dst));
	if (ret) cerr << "Error in NvBufSurfaceFromFd dst." << endl;		

	ret = NvBufSurfaceMap(nvbuf_surf_src, 0, 0, NVBUF_MAP_WRITE);
	if (ret) cerr << "Error in NvBufSurfaceMap src." << endl;		
	else
	{		
		int32_t plane = 0;

		auto virtualip_data_addr = (void*)nvbuf_surf_src->surfaceList[0].mappedAddr.addr[plane];

		auto pixelByteSize = ctx->nvBufConverter->src_fmt_bytes_per_pixel[plane];

		auto copySize = //pixelByteSize *
			nvbuf_surf_src->surfaceList[0].planeParams.pitch[plane] * 
			nvbuf_surf_src->surfaceList[0].planeParams.height[plane];
			
		if( copySize == frame->payload_size[0] )
		{
			//std::cout << " - to src full copy" << endl;	
			memcpy(virtualip_data_addr,frame->payload[0],copySize);
		}	
		else
		{
			std::cout << " - to src FAILED SIZE MISMATCH" << endl;	
		}
			
		
		NvBufSurfaceSyncForDevice(nvbuf_surf_src, 0, plane);
	}
	NvBufSurfaceUnMap(nvbuf_surf_src, 0, 0);

	ret = NvBufSurf::NvTransform(&ctx->nvBufConverter->transform_params, ctx->nvBufConverter->in_dmabuf_fd, ctx->nvBufConverter->out_dmabuf_fd);
	if (ret)
	{
		cerr << "Error in transformation." << endl;			
	}
	
	for (uint32_t plane = 0; plane < nvbuf_surf_dst->surfaceList[0].planeParams.num_planes; plane ++)
    {
		auto pixelByteSize = ctx->nvBufConverter->dest_fmt_bytes_per_pixel[plane];
		
		auto copySize = nvbuf_surf_dst->surfaceList[0].planeParams.pitch[plane] * nvbuf_surf_dst->surfaceList[0].planeParams.height[plane];
	
		payload_size[plane] =  pixelByteSize *
			nvbuf_surf_dst->surfaceList[0].planeParams.width[plane] * 
			nvbuf_surf_dst->surfaceList[0].planeParams.height[plane];

		ctx->nvBufConverter->planeConversionData[plane].resize(payload_size[plane]);
		payload[plane] = (unsigned char *)ctx->nvBufConverter->planeConversionData[plane].data();

		ret = NvBufSurfaceMap(nvbuf_surf_dst, 0, plane, NVBUF_MAP_READ);
		if (ret) cerr << "Error in NvBufSurfaceMap dst. Plane: " << plane << endl;		
		else
		{			
			NvBufSurfaceSyncForCpu (nvbuf_surf_dst, 0, plane);
			uint8_t* virtualip_data_addr = (uint8_t*)nvbuf_surf_dst->surfaceList[0].mappedAddr.addr[plane];

			if( copySize == payload_size[plane]  )
			{
				//std::cout << " - from dst full copy" << endl;					
				memcpy(ctx->nvBufConverter->planeConversionData[plane].data(), virtualip_data_addr,	payload_size[plane]);
			}	
			else
			{
				//std::cout << " - fromt dst line copy" << endl;	
				
				auto dstLineSizeSize = pixelByteSize * nvbuf_surf_dst->surfaceList[0].planeParams.width[plane];
				auto surfaceLineSize = nvbuf_surf_dst->surfaceList[0].planeParams.pitch[plane];
				uint8_t *dstAddr = (uint8_t *)ctx->nvBufConverter->planeConversionData[plane].data();
								
				for (uint32_t lineY = 0; lineY < nvbuf_surf_dst->surfaceList[0].planeParams.height[plane]; ++lineY)
				{
					memcpy(dstAddr,
						virtualip_data_addr,
						dstLineSizeSize);
						
					dstAddr += dstLineSizeSize;
					virtualip_data_addr += surfaceLineSize;
				}
			}										
		}
		NvBufSurfaceUnMap(nvbuf_surf_dst, 0, plane);        
    }

	return nvmpi_video_put_frame(ctx, payload_size, payload, frame->timestamp, nullptr);
}

#endif

int nvmpi_encoder_put_frame(nvmpictx* ctx,nvFrame* frame)
{
	// does it need an image conversion first?
#if JETPACK_VER >= 4
	if(frame->payload_size[0] > 0 && ctx->enableImageConverter )
	{
		return nvmpi_converter_put_frame(ctx, frame);
	}
#endif
	
	return nvmpi_video_put_frame(ctx, frame->payload_size, frame->payload, frame->timestamp, nullptr);
}

int nvmpi_encoder_get_packet(nvmpictx* ctx,nvPacket* packet){

	int ret,packet_index;

	if(ctx->packet_pools->empty())
		return -1;

	packet_index = ctx->packet_pools->front();

	auto ts = ctx->timestamp[packet_index];
	auto size = ctx->packets_size[packet_index];
	if((ts > 0) && (size == 0)) // Old packet, but 0-0 skip!
	{
		return -1;
	}

	packet->payload=ctx->packets[packet_index];
	packet->pts=ts;

	packet->payload_size=size;
	if(ctx->packets_keyflag[packet_index])
		packet->flags|= 0x0001;//AV_PKT_FLAG_KEY 0x0001
	ctx->packets_size[packet_index] = 0; // mark as readed
	ctx->packet_pools->pop();
	return 0;
}

int nvmpi_encoder_close(nvmpictx* ctx)
{
	std::cout << "******** CLOSE ENCODER *********" << endl;
	std::cout << "nvmpi_encoder_close: " << ctx->GUID << endl;
	
	std::cout << " - close for image processing: " << ctx->GUID << endl;

	// shutdown img converter first
#if JETPACK_VER == 4
	if(ctx->ImgPlaneConverter)
	{
		ctx->ImgPlaneConverter->ShutDown();
		ctx->ImgPlaneConverter.reset();
	}	
#elif JETPACK_VER >= 5
	ctx->nvBufConverter.reset();
#endif

	std::cout << " - close for encoder: " << ctx->GUID << endl;

	//ctx->enc->capture_plane.stopDQThread();
	ctx->enc->capture_plane.waitForDQThread(1000);
	// clear it out
	ctx->enc.reset();

	delete ctx->packet_pools;
	delete ctx;

	cout << "nvmpi_encoder_close: exit" << std::endl;

	return 0;
}

