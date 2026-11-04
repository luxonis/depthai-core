project(replay_examples)
cmake_minimum_required(VERSION 3.10)

## function: dai_add_example(example_name example_src enable_test use_pcl)
## function: dai_set_example_test_labels(example_name ...)

if(DEPTHAI_FETCH_ARTIFACTS)
    private_data(
        URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/misc/recording.tar"
        SHA1 "b1e31a26c83dc1e315132c9226097da4b1a5cbb7"
        FILE "recording.tar"
        LOCATION recording_path
    )
endif()

dai_add_example(replay_video_meta replay_video_meta.cpp OFF OFF)

dai_add_example(replay_video replay_video.cpp OFF OFF)

dai_add_example(replay_to_camera replay_to_camera.cpp OFF OFF)

dai_add_example(replay_to_processing replay_to_processing.cpp OFF OFF)

dai_add_example(replay_imu replay_imu.cpp OFF OFF)

if(DEPTHAI_FETCH_ARTIFACTS)
    # recording.tar has 119 video frames and 140 metadata messages. ReplayVideo rejects that pair.
    dai_add_example(holistic_replay holistic_replay.cpp OFF OFF)
    target_compile_definitions(holistic_replay PRIVATE RECORDING_PATH="${recording_path}")
endif()

dai_add_example(record_video record_video.cpp OFF OFF)

dai_add_example(holistic_record holistic_record.cpp OFF OFF)

dai_add_example(record_encoded record_encoded.cpp OFF OFF)

dai_add_example(record_raw record_raw.cpp OFF OFF)

dai_add_example(record_raw_frames record_raw_frames.cpp OFF OFF)

dai_add_example(record_event record_event.cpp OFF OFF)

dai_add_example(record_imu record_imu.cpp OFF OFF)
