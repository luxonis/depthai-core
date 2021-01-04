#include <chrono>
#include "depthai/depthai.hpp"

/////////////////////////
// TEST
// ImageManipNode test
////////////////////////
int main(){

    using namespace std;
    using namespace std::chrono;
    using namespace std::chrono_literals;
    
    // Create pipeline
    dai::Pipeline p;

    auto xin = p.create<dai::node::XLinkIn>();
    auto imageManip = p.create<dai::node::ImageManip>();
    auto xout = p.create<dai::node::XLinkOut>();
    
    // resize
    const int originalWidth = 640, originalHeight = 360;
    const int width = 337, height = 225;
    
    // Properties
    xin->setStreamName("in");
    xout->setStreamName("out");
    imageManip->setResize(width, height);

    // Link plugins CAM -> XLINK
    xin->out.link(imageManip->inputImage);
    imageManip->out.link(xout->input);


    // Wait 5 seconds for device
    auto wait = steady_clock::now();
    bool found;
    dai::DeviceInfo deviceInfo;

    do{
        std::tie(found, deviceInfo) = dai::Device::getFirstAvailableDevice();
        
        if(found) {
            dai::Device d(p, deviceInfo);
            d.startPipeline();

            auto in = d.getInputQueue("in");
            auto out = d.getOutputQueue("out");

            // Send 10 messages
            for(int i = 0; i < 10; i++){
                // Create 'rgb interleaved' frame
                dai::ImgFrame inFrame;
                inFrame.getData().resize(originalWidth * originalHeight * 3);
                inFrame.setWidth(originalWidth);
                inFrame.setHeight(originalHeight);
                inFrame.setType(dai::RawImgFrame::Type::RGB888p);
                
                // Send the frame
                in->send(inFrame);

                // Retrieve the resized frame
                auto outFrame = out->get<dai::ImgFrame>();

                // Check that out frame is resized
                if(outFrame->getWidth() != width || outFrame->getHeight() != height){
                    return -1;
                }
            }

            // Exit with success error code
            return 0;
        }

        this_thread::sleep_for(1s);
    } while(!found && steady_clock::now() - wait < 5s );

    // Otherwise exit with error    
    return -1;
}