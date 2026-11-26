project(replay_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

dai_add_example(replay_video replay_video.cpp OFF OFF)

dai_add_example(replay_on_camera replay_on_camera.cpp OFF OFF)

dai_add_example(replay_to_processing replay_to_processing.cpp OFF OFF)

dai_add_example(replay_imu replay_imu.cpp OFF OFF)

dai_add_example(holistic_replay holistic_replay.cpp OFF OFF)

dai_add_example(holistic_record holistic_record.cpp OFF OFF)

dai_add_example(record_basic record_basic.cpp OFF OFF)

dai_add_example(record_naming record_naming.cpp OFF OFF)

dai_add_example(record_start_stop record_start_stop.cpp OFF OFF)

dai_add_example(record_imu record_imu.cpp OFF OFF)
