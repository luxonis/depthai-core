#include <iostream>

#include "depthai/device.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"

#include "depthai-shared/generated/CameraSensorResolution.hpp"


int main(){
    using namespace std;
    cout << "Hello World!" << endl;

    dai::Pipeline p;

    auto colorCam = p.create<dai::node::ColorCamera>();
    auto xlinkOut = p.create<dai::node::XLinkOut>();
    
    xlinkOut->setStreamName("color");

    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::gen::CameraSensorResolution::THE_1080_P);
    colorCam->setInterleaved(true);
    colorCam->setCamId(0);

    colorCam->preview.link(xlinkOut->in);


    dai::Pipeline pCopy = p;
    colorCam->video.link(xlinkOut->in);

    cout << p.toJson() << endl << endl << endl;



    cout << pCopy.toJson() << endl;



    //Device d("", true);
    //d.create_pipeline("");


    return 0;
}