/*
 * Copyright 2022 Bonn-Rhein-Sieg University
 *
 * Author: Kevin Patel
 *
 */

#ifndef MIR_OBJECT_RECOGNITION_YOLO_INFERENCE_HPP
#define MIR_OBJECT_RECOGNITION_YOLO_INFERENCE_HPP

#include <opencv2/opencv.hpp>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Namespaces.
using namespace cv;
using namespace std;
using namespace cv::dnn;

// ONNX model converted for this image dimensions
// Constants.
inline const float INPUT_WIDTH = 640.0;
inline const float INPUT_HEIGHT = 640.0;
inline const float SCORE_THRESHOLD = 0.5;
inline const float NMS_THRESHOLD = 0.45;
inline const float CONFIDENCE_THRESHOLD = 0.7;

// Text parameters.
inline const float FONT_SCALE = 0.6;
inline const int FONT_FACE = FONT_HERSHEY_SIMPLEX;
inline const int THICKNESS = 1;

// Colors.
Scalar BLACK = Scalar(0, 0, 0);
Scalar BLUE = Scalar(255, 178, 50);
Scalar YELLOW = Scalar(0, 255, 255);
Scalar RED = Scalar(0, 0, 255);

struct RecognizedObject
{
  std::string class_name;
  float confidence;
  std::vector<int> roi;
};

typedef std::vector<RecognizedObject> RecognizedObjectList;


class YoloInference
{
public:
    YoloInference(std::string net_file, std::string classes_file);
    ~YoloInference();

    vector<string> class_list;
    Mat frame;
    Net net;

    RecognizedObjectList run_inference(Mat &input_image);
    void draw_label(Mat &input_image, string label, int left, int top);
    vector<Mat> pre_process(Mat &input_image);
    RecognizedObjectList post_process(Mat &input_image, vector<Mat> &outputs);
};

#endif // MIR_OBJECT_RECOGNITION_YOLO_INFERENCE_HPP