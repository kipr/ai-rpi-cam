/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * kipr_post_rpocessing_stage.hpp - IMX500 post processing stage base class
 */

#pragma once

#include <fstream>
#include <mutex>

#include <boost/property_tree/ptree.hpp>

#include <libcamera/geometry.h>
#include <libcamera/stream.h>

#include "core/completed_request.hpp"
#include "core/rpicam_app.hpp"
#include "post_processing_stages/post_processing_stage.hpp"

class KIPRPostProcessingStage : public PostProcessingStage
{
public:
	static constexpr unsigned int Max_Num_Tensors = 16;
	static constexpr unsigned int Max_Num_Dimensions = 16;
	static constexpr unsigned int Network_Name_Len = 64;
	static constexpr unsigned int NetworkNameLen = 64;

	enum class MODEL_TYPE
	{
		YOLO11n,
		nano,
		mobilnetv2
	};

#define MODEL_DEFAULT "mobilnetv2"

	const std::unordered_map<std::string, MODEL_TYPE> stringToEnumMap =
	{
		{"YOLO11n", MODEL_TYPE::YOLO11n},
		{"nano",    MODEL_TYPE::nano},
		{"mobilnetv2", MODEL_TYPE::mobilnetv2}
	};

	enum BBOX_NORMALIZATION_TYPE
	{
		YX,
		XY
	};

#define BBOX_NORMALIZATION_DEFAULT "yx"

	MODEL_TYPE model;
	BBOX_NORMALIZATION_TYPE bbox_normalization;

	struct OutputTensorInfo
	{
		uint32_t tensor_data_num;
		uint32_t num_dimensions;
		uint16_t size[Max_Num_Dimensions];
	};

	struct CnnOutputTensorInfo
	{
		char network_name[Network_Name_Len];
		uint32_t num_tensors;
		OutputTensorInfo info[Max_Num_Tensors];
	};

	struct CnnInputTensorInfo {
		char networkName[NetworkNameLen];
		uint32_t width;
		uint32_t height;
		uint32_t numChannels;
	};

#define INPUT_TENSOR_WIDTH 640
#define INPUT_TENSOR_HEIGHT 480

	uint32_t Input_Tensor_width;
	uint32_t Input_Tensor_height;

	std::string model_;
	std::string bbox_normalization_;

	KIPRPostProcessingStage(RPiCamApp *app);
	~KIPRPostProcessingStage();

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

	libcamera::Rectangle ConvertInferenceCoordinates(const std::vector<float> &coords,
													 const libcamera::Rectangle &scalerCrop) const;
	void SetInferenceRoiAbs(const libcamera::Rectangle &roi_) const;
	void SetInferenceRoiAuto(const unsigned int width, const unsigned int height) const;
	void ShowFwProgressBar();

protected:
	libcamera::Rectangle full_sensor_resolution_ = libcamera::Rectangle(0, 0, 4056, 3040);
	libcamera::Stream *output_stream_;
	libcamera::Stream *raw_stream_;

private:
	void doProgressBar();

	int device_fd_;
	std::ifstream fw_progress_;
	std::ifstream fw_progress_chunk_;

	std::ofstream input_tensor_file_;
	unsigned int num_input_tensors_saved_;
	unsigned int save_frames_;
	std::vector<int32_t> norm_val_;
	std::vector<uint8_t> norm_shift_;
	std::vector<int16_t> div_val_;
	unsigned int div_shift_;
	std::mutex lock_;
};
