#include <iostream>
#include <fstream>
#include <chrono>
#include "depthai/depthai.hpp"
#include "protos/messages.pb.h"
#include "protos/ImgDetection.pb.h"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/utility/Serialization.hpp"

  using std::chrono::high_resolution_clock;
  using std::chrono::duration_cast;
  using std::chrono::duration;
  using std::chrono::milliseconds;

void current_serialization_test(dai::ImgDetections& imgDetection){
  std::cout << "Testing current serialization and desiralization speed" << std::endl;

  dai::ImgDetections des;
  
  auto t1 = high_resolution_clock::now();

  auto ser = dai::utility::serialize(imgDetection.detections);
  dai::utility::deserialize(ser, des.detections);

  auto t2 = high_resolution_clock::now();

  // Getting number of milliseconds as an integer.
  // auto ms_int = duration_cast<milliseconds>(t2 - t1);

  //Getting number of milliseconds as a double.
  duration<double, std::milli> ms_double = t2 - t1;

  // std::cout << "Took " << ms_int.count() << "ms" << std::endl;
  std::cout << "Took " << ms_double.count() << "ms" << std::endl;
}

void protobuf_serialization_test(dai::ImgDetections& imgDetection){
  std::cout << "Testing protobuf serialization and desiralization speed" << std::endl;

  proto_speed_test::ImgDetections imgDetections_proto;

  // convert ImgDetection to ImgDetections_proto
  for(const auto& detection : imgDetection.detections) {
      auto det = imgDetections_proto.add_detections();
      det->set_label(detection.label);
      det->set_confidence(detection.confidence);
      det->set_xmin(detection.xmin);
      det->set_ymin(detection.ymin);
      det->set_xmax(detection.xmax);
      det->set_ymax(detection.ymax);
  }
  
  /*
  //proto_speed_test::ImgDetections_Detection* detection = imgDetections.add_detections();
  for (int i = 0; i < num_detections; ++i) {
      auto label = i * step * max_label;
      proto_speed_test::ImgDetections_Detection* det = imgDetections_proto.add_detections();
      det->set_label(label);
      det->set_confidence(static_cast<float>(std::rand()) / RAND_MAX);
      det->set_xmin(static_cast<float>(std::rand()) / RAND_MAX);
      det->set_ymin(static_cast<float>(std::rand()) / RAND_MAX);
      det->set_xmax(static_cast<float>(std::rand()) / RAND_MAX);
      det->set_ymax(static_cast<float>(std::rand()) / RAND_MAX);
  }*/
  
  std::string serialized;
  proto_speed_test::ImgDetections deserialized;

  auto t1 = high_resolution_clock::now();

  imgDetections_proto.SerializeToString(&serialized);
  deserialized.ParseFromString(serialized);

  auto t2 = high_resolution_clock::now();

  // Getting number of milliseconds as an integer.
  // auto ms_int = duration_cast<milliseconds>(t2 - t1);

  // Getting number of milliseconds as a double.
  duration<double, std::milli> ms_double = t2 - t1;

  // std::cout << "Took " << ms_int.count() << "ms" << std::endl;
  std::cout << "Took " << ms_double.count() << "ms" << std::endl;

  // Shutdown the protocol buffer library.
  google::protobuf::ShutdownProtobufLibrary();
}

int main(int argc, char const *argv[])
{
  std::cout << "Testing serialization and desiralization speed" << std::endl;

  // Create ImgDetection and populate with random data
  dai::ImgDetections imgDetection;
  constexpr int num_detections = 100000;
  constexpr int max_label = 9;
  constexpr float step = 1.0f / num_detections;
  for (int i = 0; i < num_detections; ++i) {
    auto label = i * step * max_label;
    dai::ImgDetection detection;
    detection.label = static_cast<uint32_t>(std::rand() % max_label);
    detection.confidence = static_cast<float>(std::rand()) / RAND_MAX;
    detection.xmin = static_cast<float>(std::rand()) / RAND_MAX;
    detection.ymin = static_cast<float>(std::rand()) / RAND_MAX;
    detection.xmax = static_cast<float>(std::rand()) / RAND_MAX;
    detection.ymax = static_cast<float>(std::rand()) / RAND_MAX;

    imgDetection.detections.push_back(detection);
  }

  //print num_detections
  std::cout << "num_detections: " << num_detections << std::endl;

  current_serialization_test(imgDetection);

  protobuf_serialization_test(imgDetection);

  return 0;
}
