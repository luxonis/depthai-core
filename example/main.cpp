#include <iostream>

#include "depthai/device.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

#include "depthai-shared/Assets.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai/pipeline/node/NeuralNetwork.hpp"



#include "depthai-shared/generated/CameraSensorResolution.hpp"
#include "depthai-shared/datatype/NNTensor.hpp"
#include "depthai-shared/datatype/ImgFrame.hpp"



#include "opencv2/opencv.hpp"


dai::Pipeline createNNPipeline(){


    dai::Pipeline p;

    auto colorCam = p.create<dai::node::ColorCamera>();
    auto xlinkOut = p.create<dai::node::XLinkOut>();
    auto nn1 = p.create<dai::node::NeuralNetwork>();

    //nn1->setBlobPath("/blob/path");

    xlinkOut->setStreamName("detections");
    

    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::gen::CameraSensorResolution::THE_1080_P);
    colorCam->setInterleaved(true);
    colorCam->setCamId(0);

    // Link plugins CAM -> NN -> XLINK
    colorCam->preview.link(nn1->in);
    nn1->out.link(xlinkOut->in);

    return p;

}

dai::Pipeline createCameraPipeline(){



    dai::Pipeline p;

    auto colorCam = p.create<dai::node::ColorCamera>();
    auto xlinkOut = p.create<dai::node::XLinkOut>();
    xlinkOut->setStreamName("preview");
    
    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::gen::CameraSensorResolution::THE_1080_P);
    colorCam->setInterleaved(true);
    colorCam->setCamId(0);

    // Link plugins CAM -> XLINK
    colorCam->preview.link(xlinkOut->in);
    
    return p;

}


dai::Pipeline createCameraFullPipeline(){



    dai::Pipeline p;

    auto colorCam = p.create<dai::node::ColorCamera>();
    auto xlinkOut = p.create<dai::node::XLinkOut>();
    xlinkOut->setStreamName("video");
    
    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::gen::CameraSensorResolution::THE_1080_P);
    colorCam->setInterleaved(true);
    colorCam->setCamId(0);

    // Link plugins CAM -> XLINK
    colorCam->video.link(xlinkOut->in);
    
    return p;

}


int main(int argc, char** argv){
    using namespace std;
    cout << "Hello World!" << endl;


    //dai::Device d();
    //sleep(5);

    dai::Pipeline p = createCameraFullPipeline();
    //cout << p.toJson().dump(4) << endl << endl << endl;

    /*
    
    dai::AssetManager assetManager;
    p.loadAssets(assetManager);

    cout << p.toJson().dump(4) << endl << endl << endl;

    nlohmann::json jassets = assetManager;
    cout << jassets.dump(4) << endl << endl << endl;


    for(const auto& b : assetManager.serialize()){
        printf("%02X ", b);
    }
    cout << endl << endl;

    //cout << pCopy.toJson() << endl;
    
    */



    bool found;
    dai::DeviceInfo deviceInfo;
    std::tie(found, deviceInfo) = dai::XLinkConnection::getFirstDevice(X_LINK_BOOTED);

    if(found) {
        dai::Device d(deviceInfo);

        d.startPipeline(p);

        
        cv::Mat frame;
        auto& preview = d.getOutputQueue("video");
        while(1){
            
            auto imgFrame = preview.get<dai::ImgFrame>();
            if(imgFrame){
                printf("Frame - w: %d, h: %d\n", imgFrame->fb.width, imgFrame->fb.height);

                frame = cv::Mat(imgFrame->fb.height * 3 / 2, imgFrame->fb.width, CV_8UC1, imgFrame->data.data());
                
                cv::Mat rgb(imgFrame->fb.height, imgFrame->fb.width, CV_8UC3);

                cv::cvtColor(frame, rgb, cv::COLOR_YUV2BGR_NV12);
                cv::imshow("video", rgb);
                cv::waitKey(1);
            } else {
                std::cout << "Not ImgFrame" << std::endl;
            }
            
        }


        /*


        // start a while loop for getting em detections
        auto& nnQueue = d.getOutputQueue("detections");
        while(1){
            if( nnQueue.has<dai::NNTensor>() ){

                auto result = nnQueue.get<dai::NNTensor>();

                cout << "Tensors: ";        
                for(const auto& t : result->tensors){
                    cout << "'" + t.name + "'" << ", ";
                }                            
                cout << "Data: ";
                for(const auto& b : result->data){
                    printf("%02X ", b);
                }    
                cout << endl;

            } else {
                
                auto data = nnQueue.get();
                       
                cout << "Data: ";
                for(const auto& b : data->data){
                    printf("%02X ", b);
                }    
                cout << endl;
            }


        }

        */

    } else {
        cout << "No booted (debugger) devices found..." << endl;
    }




    //Device d("", true);
    //d.create_pipeline("");


    return 0;
}