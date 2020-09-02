#include <iostream>

#include "depthai/device.hpp"
#include "depthai-shared/Assets.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai/pipeline/node/NeuralNetwork.hpp"


#include "depthai-shared/generated/CameraSensorResolution.hpp"



int main(int argc, char** argv){
    using namespace std;
    cout << "Hello World!" << endl;


    //dai::Device d();
    //sleep(5);



    dai::Pipeline p;

    auto colorCam = p.create<dai::node::ColorCamera>();
    auto xlinkOut = p.create<dai::node::XLinkOut>();
    auto nn1 = p.create<dai::node::NeuralNetwork>();
    auto nn2 = p.create<dai::node::NeuralNetwork>();

    nn1->setBlobPath("test.txt");
    nn2->setBlobPath("test2.txt");

    xlinkOut->setStreamName("color");
    

    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::gen::CameraSensorResolution::THE_1080_P);
    colorCam->setInterleaved(true);
    colorCam->setCamId(0);

    colorCam->preview.link(xlinkOut->in);


    dai::Pipeline pCopy = p;
    colorCam->video.link(xlinkOut->in);


    
    dai::AssetManager assetManager;
    p.loadAssets(assetManager);

    cout << p.toJson().dump(4) << endl << endl << endl;

    nlohmann::json jassets = assetManager;
    cout << jassets.dump(4) << endl << endl << endl;


    for(const auto& b : assetManager.serialize()){
        printf("%02X ", b);
    }
    cout << endl << endl;

    cout << pCopy.toJson() << endl;
    






    //Device d("", true);
    //d.create_pipeline("");


    return 0;
}