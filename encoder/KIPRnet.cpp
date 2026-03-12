

#include "encoder/KIPRnet.hpp"

#include "post_processing_stages/object_detect.hpp"

#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/opencv.hpp>

extern "C"
{
#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libavutil/avutil.h"
#include "libswscale/swscale.h"
#include <libavutil/imgutils.h>
}

#include "PracticalSocket.hpp"

using namespace std;

/*debug */void HexDump(const void* mem, unsigned int size);


#define VIDEO_CODEC "libx264"

KIPRnet::KIPRnet() : abort_video_(false)
{
	video_thread_ = std::thread(&KIPRnet::videoThread, this);

	const AVCodec *codec = avcodec_find_encoder_by_name(VIDEO_CODEC);

        if (!codec)
                throw std::runtime_error("libav: cannot find video encoder " );

        codec_ctx_[Video] = avcodec_alloc_context3(codec);
        if (!codec_ctx_[Video])
                throw std::runtime_error("libav: Cannot allocate video context");

//        codec_ctx_[Video]->width = info.width;
//        codec_ctx_[Video]->height = info.height;
        // usec timebase
        codec_ctx_[Video]->time_base = { 1, 1000 * 1000 };
        codec_ctx_[Video]->sw_pix_fmt = AV_PIX_FMT_YUV420P;
        codec_ctx_[Video]->pix_fmt = AV_PIX_FMT_YUV420P;

	header_count = 0;

	LOG(2, "KIPRnet init completed");
}

KIPRnet::~KIPRnet()
{
	abort_video_ = true;
	video_thread_.join();
}

bool KIPRnet::KIPRencode(CompletedRequestPtr &completed_request, Stream *stream)
{
	StreamInfo info = GetStreamInfo(stream);
	FrameBuffer *buffer = completed_request->buffers[stream];
	BufferReadSync r(this, buffer);
	libcamera::Span span = r.Get()[0];
	void *mem = span.data();
	if (!buffer || !mem)
		throw std::runtime_error("no buffer to encode");
	auto ts = completed_request->metadata.get(controls::SensorTimestamp);
	int64_t timestamp_ns = ts ? *ts : buffer->metadata().timestamp;
#ifdef NONEEDED
	{
		std::lock_guard<std::mutex> lock(encode_buffer_queue_mutex_);
		encode_buffer_queue_.push(completed_request); // creates a new reference
	}
#endif

	/* get detected objects */

        std::vector<Detection> detections;

        completed_request->post_process_metadata.Get("object_detect.results", detections);

	int object_count = detections.size();

	header_buf * header_buff = new header_buf;
	header_buff->length = (2 * sizeof(int)) + (object_count * sizeof(object_detection));
	header_buff->buffer = new char[header_buff->length];

	header_count++;
	header_buff->header_seq = header_count;

cout << "KIPRencode - objects found: " << object_count << " Header size: " << header_buff->length << " header_seq: " << header_count << endl;

	memcpy( &header_buff->buffer[sizeof(int)], &object_count, sizeof(int));

	object_detection * object_struct;

	int count = 0;

        for (auto &detection : detections)
        {
		object_struct = (object_detection *) &header_buff->buffer[2*sizeof(int) + (count * sizeof(object_detection))];

		object_struct->category = detection.category;
		object_struct->confidence = detection.confidence;
		object_struct->box.x = detection.box.x;
		object_struct->box.width = detection.box.width;
		object_struct->box.y = detection.box.y;
		object_struct->box.height = detection.box.height;

                cout << detection.toString() << endl;
		int name_size = detection.name.size();
		if(name_size > MAX_NAME_LEN)
			name_size = MAX_NAME_LEN;

		memcpy(&object_struct->name, detection.name.c_str(), detection.name.size());
		object_struct->name_len = name_size;

		count++;

        }

	EncodeBuffer(buffer->planes()[0].fd.get(), span.size(), mem, info, timestamp_ns / 1000, header_buff);

	// Tell our caller that encoding is underway.
	return true;
}

void KIPRnet::EncodeBuffer(int fd, size_t size, void *mem, StreamInfo const &info, int64_t timestamp_us, header_buf* header_buf)
{
#ifdef OBJDECTION
	//std::vector<Detection> detections;

	//completed_request->post_process_metadata.Get("object_detect.results", detections);

#endif
	AVFrame *frame = av_frame_alloc();
	if (!frame)
		throw std::runtime_error("libav: could not allocate AVFrame");
	if (!video_start_ts_)
		video_start_ts_ = timestamp_us;

	frame->format = codec_ctx_[Video]->pix_fmt;
	frame->width = info.width;
	frame->height = info.height;
	frame->linesize[0] = info.stride;
	frame->linesize[1] = frame->linesize[2] = info.stride >> 1;
	frame->pts = timestamp_us - video_start_ts_ ;

	frame->buf[0] = av_buffer_create((uint8_t *)mem, size, &KIPRnet::releaseBuffer, this, 0);
	av_image_fill_pointers(frame->data, AV_PIX_FMT_YUV420P, frame->height, frame->buf[0]->data, frame->linesize);
	av_frame_make_writable(frame);


	std::scoped_lock<std::mutex> lock(video_mutex_);
	while(!frame_queue_.empty())
	{	
		frame_queue_.pop();
	}
	while(!header_queue_.empty())
	{
		header_queue_.pop();
	}
	frame_queue_.push(frame);
	header_queue_.push(header_buf);
	video_cv_.notify_all();
}

/* used frame by the sw detect emech */
#define FRAME_HEIGHT 480
#define FRAME_WIDTH 640
#define FRAME_INTERVAL (1000 / 30)
#define PACK_SIZE 4096 //udp pack size; note that OSX limits < 8100 bytes
#define ENCODE_QUALITY 95

void KIPRnet::videoThread()
{
	//AVPacket *pkt = av_packet_alloc();
	AVFrame *frameAV = nullptr;
	struct header_buf * header_buffer = nullptr;

	while (true)
	{
		{
			std::unique_lock<std::mutex> lock(video_mutex_);
			while (true)
			{
				using namespace std::chrono_literals;
				// Must check the abort first, to allow items in the output
				// queue to have a callback.
				if (abort_video_ && frame_queue_.empty())
					goto done;

				if (!frame_queue_.empty())
				{
					frameAV = frame_queue_.front();
					frame_queue_.pop();
					header_buffer = header_queue_.front();
					header_queue_.pop();
					break;
				}
				else
					video_cv_.wait_for(lock, 200ms);
			}
		}

		//convert AVFrame to opencv CV::MAT

		//************************************
		// Method:    avframeToCvmat
		// Access:    public
		// Returns:   cv::Mat
		// Qualifier:
		// Parameter: const AVFrame * frame
		// Description: AVFrame转MAT
		//**********************************

		int width = frameAV->width;
		int height = frameAV->height;
		cv::Mat frameMAT(height, width, CV_8UC3), send;
		std::vector<uchar> encoded;
		int cvLinesizes[1];
		cvLinesizes[0] = frameMAT.step1();

		SwsContext *conversion = sws_getContext(width, height, (AVPixelFormat)frameAV->format, width, height,
							AVPixelFormat::AV_PIX_FMT_BGR24, SWS_FAST_BILINEAR, NULL, NULL, NULL);
		sws_scale(conversion, frameAV->data, frameAV->linesize, 0, height, &frameMAT.data, cvLinesizes);
		sws_freeContext(conversion);

		int jpegqual = ENCODE_QUALITY; // Compression Parameter
		UDPSocket sock;

		if (frameMAT.size().width == 0)
			continue; //simple integrity check; skip erroneous data...

		resize(frameMAT, send, cv::Size(FRAME_WIDTH, FRAME_HEIGHT), 0, 0, cv::INTER_LINEAR);
		std::vector<int> compression_params;
		compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
		compression_params.push_back(jpegqual);
		imencode(".jpg", send, encoded, compression_params);
		//imencode(".png", send, encoded, compression_params);

		int total_pack = 1 + (encoded.size() - 1) / PACK_SIZE;
		int image_size = encoded.size();

/*debug*/       int object_count;
		memcpy(&object_count, header_buffer->buffer + sizeof(int), sizeof(int));

		//memcpy(header_buffer->buffer, &total_pack, sizeof(int));
		memcpy(header_buffer->buffer, &image_size, sizeof(int));

		sock.sendTo(header_buffer->buffer, header_buffer->length, "192.168.1.1", 9000);
		cout << "KIPR - sending frame " << total_pack << " object_count: " << object_count << " Header size: " << header_buffer->length << " header_seq " << header_buffer->header_seq << "image_size: " << image_size << endl;

/*DEBUG*/	//HexDump(&encoded[0], 16);
		if (video_output == 1)
		{
			for (int i = 0; i < total_pack; i++)
			{
				int packet_size;
				packet_size = PACK_SIZE;

				sock.sendTo(&encoded[i * PACK_SIZE], packet_size, "192.168.1.1", 9000);
			}
		}
		av_frame_free(&frameAV);
		free(header_buffer->buffer);
		free(header_buffer);
	}

done:
	deinitOutput();
}

void KIPRnet::deinitOutput()
{
}

extern "C" void KIPRnet::releaseBuffer(void *opaque, uint8_t *data)
{
#ifdef NOTUSED
	LibAvEncoder *enc = static_cast<LibAvEncoder *>(opaque);

	enc->input_done_callback_(nullptr);

	// Pop the entry from the queue to release the AVDRMFrameDescriptor allocation
	std::scoped_lock<std::mutex> lock(enc->drm_queue_lock_);
	if (!enc->drm_frame_queue_.empty())
		enc->drm_frame_queue_.pop();
#endif
}
/*DEBUG remove for production */

void HexDump(const void* mem, unsigned int size) {
    // Cast the void pointer to an unsigned char pointer for byte-level access
    const unsigned char* p = reinterpret_cast<const unsigned char*>(mem);

    for (unsigned int i = 0; i < size; ++i) {
        // Print the byte in hexadecimal format, with a width of 2 and leading zeros
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned int>(p[i]) << " ";

        // Add a newline every 16 bytes for better readability
        if ((i + 1) % 16 == 0) {
            std::cout << std::endl;
        }
    }
    std::cout << std::dec << std::endl; // Reset to decimal output
}

