/**
 * This example shows usage of Camera Control message as well as ColorCamera configInput to change crop x and y
 * Uses 'WASD' controls to move the crop window, 'C' to capture a still image, 'T' to trigger autofocus, 'IOKL,.'
 * for manual exposure/focus:
 *   Control:      key[dec/inc]  min..max
 *   exposure time:     I   O      1..33000 [us]
 *   sensitivity iso:   K   L    100..1600
 *   focus:             ,   .      0..255 [far..near]
 * To go back to auto controls:
 *   'E' - autoexposure
 *   'F' - autofocus (continuous)
 */
#include <iostream>
#include <cstdio>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"


// Step size ('W','A','S','D' controls)
static constexpr int STEP_SIZE = 16;

// Manual exposure/focus set step
static constexpr int EXP_STEP = 500;  // us
static constexpr int ISO_STEP = 50;
static constexpr int LENS_STEP = 3;

static int clamp(int num, int v0, int v1) {
    return std::max(v0, std::min(num, v1));
}

int main(){

    dai::Pipeline pipeline;

    // Nodes
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto controlIn = pipeline.create<dai::node::XLinkIn>();
    auto configIn = pipeline.create<dai::node::XLinkIn>();
    auto videoEncoder = pipeline.create<dai::node::VideoEncoder>();
    auto stillEncoder = pipeline.create<dai::node::VideoEncoder>();
    auto videoMjpegOut = pipeline.create<dai::node::XLinkOut>();
    auto stillMjpegOut = pipeline.create<dai::node::XLinkOut>();
    auto previewOut = pipeline.create<dai::node::XLinkOut>();

    // Properties
    colorCam->setVideoSize(640, 360);
    colorCam->setPreviewSize(300, 300);
    controlIn->setStreamName("control");
    configIn->setStreamName("config");
    videoEncoder->setDefaultProfilePreset(colorCam->getVideoSize(), colorCam->getFps(), dai::VideoEncoderProperties::Profile::MJPEG);
    stillEncoder->setDefaultProfilePreset(colorCam->getStillSize(), 1, dai::VideoEncoderProperties::Profile::MJPEG);
    videoMjpegOut->setStreamName("video");
    stillMjpegOut->setStreamName("still");
    previewOut->setStreamName("preview");

    // Link nodes
    colorCam->video.link(videoEncoder->input);
    colorCam->still.link(stillEncoder->input);
    colorCam->preview.link(previewOut->input);
    controlIn->out.link(colorCam->inputControl);
    configIn->out.link(colorCam->inputConfig);
    videoEncoder->bitstream.link(videoMjpegOut->input);
    stillEncoder->bitstream.link(stillMjpegOut->input);

    // Connect to device
    dai::Device dev(pipeline);
    
    // Create data queues
    auto controlQueue = dev.getInputQueue("control");
    auto configQueue = dev.getInputQueue("config");
    auto previewQueue = dev.getOutputQueue("preview");
    auto videoQueue = dev.getOutputQueue("video");
    auto stillQueue = dev.getOutputQueue("still");

    // Start pipeline
    dev.startPipeline();

    // Max crop_x & crop_y
    float max_crop_x = (colorCam->getResolutionWidth() - colorCam->getVideoWidth()) / (float) colorCam->getResolutionWidth();
    float max_crop_y = (colorCam->getResolutionHeight() - colorCam->getVideoHeight()) / (float) colorCam->getResolutionHeight();

    // Default crop
    float crop_x = 0;
    float crop_y = 0;

    // Defaults and limits for manual focus/exposure controls
    int lens_pos = 150;
    int lens_min = 0;
    int lens_max = 255;

    int exp_time = 20000;
    int exp_min = 1;
    int exp_max = 33000;

    int sens_iso = 800;
    int sens_min = 100;
    int sens_max = 1600;

    while(true){

        auto previewFrames = previewQueue->tryGetAll<dai::ImgFrame>();
        for(const auto& previewFrame : previewFrames){
            cv::Mat frame(previewFrame->getHeight(), previewFrame->getWidth(), CV_8UC3, previewFrame->getData().data());
            cv::imshow("preview", frame);
        }
        
        auto videoFrames = videoQueue->tryGetAll<dai::ImgFrame>();
        for(const auto& videoFrame : videoFrames){

            // Decode JPEG
            auto frame = cv::imdecode(videoFrame->getData(), cv::IMREAD_UNCHANGED);
            // Display
            cv::imshow("video", frame);

        }

        auto stillFrames = stillQueue->tryGetAll<dai::ImgFrame>();
        for(const auto& stillFrame : stillFrames){
            // Decode JPEG
            auto frame = cv::imdecode(stillFrame->getData(), cv::IMREAD_UNCHANGED);
            // Display
            cv::imshow("still", frame);
        }


        // Update screen (10ms pooling rate)
        int key = cv::waitKey(10);
        if (key == 'q') {
            break;
        } else if(key == 'c'){
            dai::CameraControl ctrl;
            ctrl.setCaptureStill(true);
            controlQueue->send(ctrl);
        } else if (key == 't') {
            printf("Autofocus trigger (and disable continuous)\n");
            dai::CameraControl ctrl;
            ctrl.setAutoFocusMode(dai::CameraControl::AutoFocusMode::AUTO);
            ctrl.setAutoFocusTrigger();
            controlQueue->send(ctrl);
        } else if (key == 'f') {
            printf("Autofocus enable, continuous\n");
            dai::CameraControl ctrl;
            ctrl.setAutoFocusMode(dai::CameraControl::AutoFocusMode::CONTINUOUS_VIDEO);
            controlQueue->send(ctrl);
        } else if (key == 'e') {
            printf("Autoexposure enable\n");
            dai::CameraControl ctrl;
            ctrl.setAutoExposureEnable();
            controlQueue->send(ctrl);
        } else if (key == ',' || key == '.') {
            if (key == ',') lens_pos -= LENS_STEP;
            if (key == '.') lens_pos += LENS_STEP;
            lens_pos = clamp(lens_pos, lens_min, lens_max);
            printf("Setting manual focus, lens position: %d\n", lens_pos);
            dai::CameraControl ctrl;
            ctrl.setManualFocus(lens_pos);
            controlQueue->send(ctrl);
        } else if (key == 'i' || key == 'o' || key == 'k' || key == 'l') {
            if (key == 'i') exp_time -= EXP_STEP;
            if (key == 'o') exp_time += EXP_STEP;
            if (key == 'k') sens_iso -= ISO_STEP;
            if (key == 'l') sens_iso += ISO_STEP;
            exp_time = clamp(exp_time, exp_min, exp_max);
            sens_iso = clamp(sens_iso, sens_min, sens_max);
            printf("Setting manual exposure, time %d us, iso %d\n", exp_time, sens_iso);
            dai::CameraControl ctrl;
            ctrl.setManualExposure(exp_time, sens_iso);
            controlQueue->send(ctrl);
        } else if (key == 'w' || key == 'a' || key == 's' || key == 'd') {
            if(key == 'a'){
                crop_x -= (max_crop_x / colorCam->getResolutionWidth()) * STEP_SIZE;
                if (crop_x < 0) crop_x = max_crop_x;
            } else if(key == 'd'){
                crop_x += (max_crop_x / colorCam->getResolutionWidth()) * STEP_SIZE;
                if (crop_x > max_crop_x) crop_x = 0.0f;
            } else if(key == 'w'){
                crop_y -= (max_crop_y / colorCam->getResolutionHeight()) * STEP_SIZE;
                if (crop_y < 0) crop_y = max_crop_y;
            } else if(key == 's'){
                crop_y += (max_crop_y / colorCam->getResolutionHeight()) * STEP_SIZE;
                if (crop_y > max_crop_y) crop_y = 0.0f;
            }

            // Send new cfg to camera
            dai::ImageManipConfig cfg;
            cfg.setCropRect(crop_x, crop_y, 0, 0);
            configQueue->send(cfg);
            printf("Sending new crop - x: %f, y: %f\n", crop_x, crop_y);
        }

    }


}
