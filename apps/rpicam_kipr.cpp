/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpicam_vid.cpp - libcamera video record app.
 */

#include <chrono>
#include <poll.h>
#include <signal.h>
#include <sys/signalfd.h>
#include <sys/stat.h>
#include <string>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#include "encoder/KIPRnet.hpp"
#include "core/rpicam_encoder.hpp"
#include "output/output.hpp"

#include <opencv4/opencv2/opencv.hpp>

#define AI_VIDEO_COMMAND_PORT 5556
#define MAX_UDP_BUFF_LEN 4096

using namespace std::placeholders;

// Some keypress/signal handling.

KIPRnet app;

void HandleTCPClient(int connfd);

static int signal_received;
static void default_signal_handler(int signal_number)
{
	signal_received = signal_number;
	LOG(1, "Received signal " << signal_number);
}

static int get_key_or_signal(VideoOptions const *options, pollfd p[1])
{
	int key = 0;
	if (signal_received == SIGINT)
		return 'x';
	if (options->Get().keypress)
	{
		poll(p, 1, 0);
		if (p[0].revents & POLLIN)
		{
			char *user_string = nullptr;
			size_t len;
			[[maybe_unused]] size_t r = getline(&user_string, &len, stdin);
			key = user_string[0];
		}
	}
	if (options->Get().signal)
	{
		if (signal_received == SIGUSR1)
			key = '\n';
		else if ((signal_received == SIGUSR2) || (signal_received == SIGPIPE))
			key = 'x';
		signal_received = 0;
	}
	return key;
}

static int get_colourspace_flags(std::string const &codec)
{
	if (codec == "mjpeg" || codec == "yuv420")
		return RPiCamEncoder::FLAG_VIDEO_JPEG_COLOURSPACE;
	else
		return RPiCamEncoder::FLAG_VIDEO_NONE;
}


// The main even loop for the application.

static void event_loop(KIPRnet &app)
{
	VideoOptions const *options = app.GetOptions();
	std::unique_ptr<Output> output = std::unique_ptr<Output>(Output::Create(options));
	app.SetEncodeOutputReadyCallback(std::bind(&Output::OutputReady, output.get(), _1, _2, _3, _4));
	app.SetMetadataReadyCallback(std::bind(&Output::MetadataReady, output.get(), _1));

	app.OpenCamera();
	app.ConfigureVideo(get_colourspace_flags(options->Get().codec));
	app.StartEncoder();
	app.StartCamera();
	auto start_time = std::chrono::high_resolution_clock::now();

	// Monitoring for keypresses and signals.
	signal(SIGUSR1, default_signal_handler);
	signal(SIGUSR2, default_signal_handler);
	signal(SIGINT, default_signal_handler);
	// SIGPIPE gets raised when trying to write to an already closed socket. This can happen, when
	// you're using TCP to stream to VLC and the user presses the stop button in VLC. Catching the
	// signal to be able to react on it, otherwise the app terminates.
	signal(SIGPIPE, default_signal_handler);
	pollfd p[1] = { { STDIN_FILENO, POLLIN, 0 } };

	for (unsigned int count = 0; ; count++)
	{
		RPiCamEncoder::Msg msg = app.Wait();
		if (msg.type == RPiCamApp::MsgType::Timeout)
		{
			LOG_ERROR("ERROR: Device timeout detected, attempting a restart!!!");
			app.StopCamera();
			app.StartCamera();
			continue;
		}
		if (msg.type == RPiCamEncoder::MsgType::Quit)
			return;
		else if (msg.type != RPiCamEncoder::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");
		int key = get_key_or_signal(options, p);
		if (key == '\n')
			output->Signal();

		LOG(2, "Viewfinder frame " << count);
		auto now = std::chrono::high_resolution_clock::now();
		bool timeout = !options->Get().frames && options->Get().timeout &&
					   ((now - start_time) > options->Get().timeout.value);
		bool frameout = options->Get().frames && count >= options->Get().frames;
		if (timeout || frameout || key == 'x' || key == 'X')
		{
			if (timeout)
				LOG(1, "Halting: reached timeout of " << options->Get().timeout.get<std::chrono::milliseconds>()
													  << " milliseconds.");
			LOG(1, "Stopping AI camera interface"); fflush(NULL);
			app.StopCamera(); // stop complains if encoder very slow to close
			app.StopEncoder();
			return;
		}
		CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);
		if (!app.KIPRencode(completed_request, app.VideoStream()))
		{
			// Keep advancing our "start time" if we're still waiting to start recording (e.g.
			// waiting for synchronisation with another camera).
			start_time = now;
			count = 0; // reset the "frames encoded" counter too
		}
		app.ShowPreview(completed_request, app.VideoStream());
	}
}

void CommandLoop();
std::thread command_thread_;

void startCommandLoop()
{
	command_thread_ = std::thread(&CommandLoop);
	LOG(1, "Command Loop Thread started");
}

void CommandLoop()
{
	int sockfd, connfd;

	in_addr in_addr_any;

	socklen_t len;	
	struct sockaddr_in servaddr, cli;

	// create the socket

	sockfd = socket(AF_INET, SOCK_STREAM, 0);

	if(sockfd == -1)
	{
		LOG_ERROR("CommandLoop - server socket creation failed");
		return;
	}

	bzero(&servaddr, sizeof(servaddr));

	in_addr_any.s_addr = INADDR_ANY;

	servaddr.sin_family = AF_INET;
	servaddr.sin_addr = in_addr_any; //htonl(in_addr_any);
	servaddr.sin_port = htons(AI_VIDEO_COMMAND_PORT);

	if((bind(sockfd, (struct sockaddr*) &servaddr, sizeof(servaddr))) != 0)
	{
		LOG_ERROR("CommandLoop - server socket bind failed");
		perror("CommandLoop - ");
		return;
	}

	if((listen(sockfd, 5)) != 0)
	{
		LOG_ERROR("CommandLoop - server socket listen failed");
		perror("CommandLoop - ");
		return;
	}

	LOG(1, "command server ready");
	len = sizeof(cli);

	for (;;)
	{
		connfd = accept(sockfd, (struct sockaddr *) &cli, &len);
		if(connfd < 0)
		{
			LOG_ERROR("CommandLoop - server accept failed");
			perror("CommandLoop - ");
			continue;
		}


		HandleTCPClient(connfd);
	}
}

std::string getForeignAddress(int sockDesc) 
    {
  sockaddr_in addr;
  unsigned int addr_len = sizeof(addr);

  if (getpeername(sockDesc, (sockaddr *) &addr,(socklen_t *) &addr_len) < 0) {
    LOG_ERROR("Fetch of foreign address failed (getpeername())");
  }
  return inet_ntoa(addr.sin_addr);
}

unsigned short getForeignPort(int sockDesc) {
  sockaddr_in addr;
  unsigned int addr_len = sizeof(addr);

  if (getpeername(sockDesc, (sockaddr *) &addr, (socklen_t *) &addr_len) < 0) {
    LOG_ERROR("Fetch of foreign port failed (getpeername())");
  }
  return ntohs(addr.sin_port);
}


void HandleTCPClient(int connfd)
{

        char buffer[MAX_UDP_BUFF_LEN]; // receive buffer
	int flag = 1;

	LOG(1, "Handling client: " << getForeignAddress(connfd) << ":" << getForeignPort(connfd) << ":");

	bzero(buffer, MAX_UDP_BUFF_LEN);

	setsockopt(connfd, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int));
for(;;)
{

	if(recv(connfd, buffer, sizeof(buffer) - 1, 0) <= 0)
	{
		close(connfd);
		perror("HandleTCPClient - recv");
		return;
	}

	LOG(1, "Received command:" << buffer);

	if(strncmp( buffer, "startvideo", 10) == 0)
	{
		int retval;
		retval = app.set_video_output(1);
		send(connfd, buffer, strnlen(buffer, MAX_UDP_BUFF_LEN), 0);

		LOG(1, "******video output started " << retval);
		fflush(NULL);
	}
	if(strncmp( buffer, "stopvideo", 10) == 0)
	{
		app.set_video_output(0);
                send(connfd, buffer, strnlen(buffer, MAX_UDP_BUFF_LEN), 0);
		LOG(1, "video output stopped");
	}
	if(strncmp( buffer, "status", 6 ) == 0)
	{
		LOG(1, "status requested");
		send(connfd, "ready", 5, 0);
		LOG(1, "status ready sent:");
		fflush(NULL);
	}
}
}

int main(int argc, char *argv[])
{

	// initialize command variables

/*DEBUG*/std::cout << cv::getBuildInformation() << std::endl;

	try
	{
//	RPiCamEncoder app;
		VideoOptions *options = app.GetOptions();

		startCommandLoop();

		if (options->Parse(argc, argv))
		{
			if (options->Get().verbose >= 2)
				options->Print();

			event_loop(app);
		}
	}
	catch (std::exception const &e)
	{
		LOG_ERROR("ERROR: *** " << e.what() << " ***");
		return -1;
	}
	return 0;
}
