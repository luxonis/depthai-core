#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include <opencv2/opencv.hpp>
#include "depthai-shared/common/Point2f.hpp"

#include <Eigen/Core>
#include <Eigen/Eigen>
int TERM_COUNT = 5;

// template<typename Scalar, typename Container>
// inline static Eigen::Matrix<Scalar,-1,-1> toEigenMatrix( const Container& vectors ){
// 	typedef typename Container::value_type VectorType;
// 	typedef typename VectorType::value_type Scalar;
// 	Eigen::Matrix<Scalar,-1,-1> M(vectors.size(), vectors.front().size());
// 	for(size_t i = 0; i < vectors.size(); i++)
// 		for(size_t j = 0; j < vectors.front().size(); j++)
// 			M(i,j) = vectors[i][j];
// 	return M;
// }
class  TermCriteria
{
public:
    /**
      Criteria type, can be one of: COUNT, EPS or COUNT + EPS
    */
    enum Type
    {
        COUNT=1, //!< the maximum number of iterations or elements to compute
        MAX_ITER=COUNT, //!< ditto
        EPS=2 //!< the desired accuracy or change in parameters at which the iterative algorithm stops
    };

    //! default constructor
    TermCriteria() : type(0), maxCount(0), epsilon(0) {}
    /**
    @param type The type of termination criteria, one of TermCriteria::Type
    @param maxCount The maximum number of iterations or elements to compute.
    @param epsilon The desired accuracy or change in parameters at which the iterative algorithm stops.
    */
    TermCriteria(int _type, int _maxCount, double _epsilon) : type(_type), maxCount(_maxCount), epsilon(_epsilon) {}

    inline bool isValid() const
    {
        const bool isCount = (type & COUNT) && maxCount > 0;
        const bool isEps = (type & EPS) && !cvIsNaN(epsilon);
        return isCount || isEps;
    }

    int type; //!< the type of termination criteria: COUNT, EPS or COUNT + EPS
    int maxCount; //!< the maximum number of iterations/elements
    double epsilon; //!< the desired accuracy
};




struct Rect{
    float x, y; // Center of the rectangle
    float width, height; // Width and height of the rectangle

    Rect(float x, float y, float width, float height){
        this->x = x;
        this->y = y;
        this->width = width;
        this->height = height;
    }
    Rect(){
        this->x = 0;
        this->y = 0;
        this->width = 0;
        this->height = 0;
    }
};

inline Rect& operator&=(Rect& lhs, const Rect& rhs)
{
    int x1 = std::max(lhs.x, rhs.x);
    int y1 = std::max(lhs.y, rhs.y);
    lhs.width  = std::min(lhs.x + lhs.width,  rhs.x + rhs.width) -  x1;
    lhs.height = std::min(lhs.y + lhs.height, rhs.y + rhs.height) - y1;
    lhs.x = x1;
    lhs.y = y1;
    if( lhs.width <= 0 || lhs.height <= 0 )
        lhs = Rect();
    return lhs;
}

void undistortPoints(std::vector<dai::Point2f>& src, std::vector<dai::Point2f>& dst, 
                Eigen::Matrix3d& M, Eigen::VectorXd& d, Eigen::Matrix3d& R, Eigen::Matrix3d& Mp, TermCriteria criteria){
 
    if (src.size() == 0)  throw std::runtime_error("Input Points were empty");
    if (M(0, 0) == 0 || M(1, 1) == 0 || M(0, 2) == 0 || M(1, 2) == 0)  throw std::runtime_error("fx, fy, cx or cy is zero. Please cross check your Camera/Intrinsics Matrix");
    if (d.size() == 14){
        if(d(12) != 0 || d(13) != 0){
            std::cout << "Tilt projections not implemented";
        }
    }
    else if (d.size() >= 4 && d.size() < 14){
        int fillOffset = 14 - d.size();
        d.resize(14);
        d.block(d.size(), 1, fillOffset, 1) = Eigen::VectorXd::Zero(fillOffset); // TODO(saching): Cross check if it is working as expected
        // d.block<fillOffset, 1>(d.size(), 1) = Eigen::VectorXd::Zero(fillOffset);
    }
    else throw std::runtime_error("distortion Coefficients are incomplete.  distortion coeffs -> (k1,k2,p1,p2[,k3[,k4,k5,k6[,s1,s2,s3,s4[,τx,τy]]]]");

    // if(d.size() == 0){
    //     d.resize(14);
    //     d = Eigen::VectorXd::Zero(14);
    // }
    Eigen::Matrix3d matTilt = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d invMatTilt = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d proj;
    if (Mp != Eigen::Matrix3d::Identity()) proj = Mp*R;
    else proj = R;

    double fx = M(0, 0);
    double fy = M(1, 1);
    double ifx = 1./fx;
    double ify = 1./fy;
    double cx = M(0, 2);
    double cy = M(1, 2);
    std::vector<dai::Point2f> internalDst;
    internalDst.reserve(src.size());

    for (unsigned int i = 0; i < src.size(); i++){
        double x, y, x0 = 0, y0 = 0, u, v;
        x = src[i].x;
        y = src[i].y;

        u = x; v = y;
        x = (x - cx)*ifx;
        y = (y - cy)*ify;

        if(d.size() >= 4){
            // distortion coeffs -> (k1,k2,p1,p2[,k3[,k4,k5,k6[,s1,s2,s3,s4[,τx,τy]]]]

            // compensate tilt distortion. 
            // Since We are not doing tilt distortions. this is set to identity. No effect on values. 
            Eigen::Vector3d vecUntilt = invMatTilt * Eigen::Vector3d(x, y, 1.0);
            double invProj = vecUntilt(2) ? 1./vecUntilt(2) : 1; // If greater than 1. use inverse ?
            x0 = x = invProj * vecUntilt(0);
            y0 = y = invProj * vecUntilt(1);
            // ---------------compensate tilt distortion-----------------------------
            double error = std::numeric_limits<double>::max();

            // compensate distortion iteratively. WHy iteratively ? We do it iteratively over x0 and y0 to get beter r^2
            for(int j = 0; ; j++){
                
                if ((criteria.type & TermCriteria::COUNT) && j >= criteria.maxCount)
                    break;
                if ((criteria.type & TermCriteria::EPS) && error < criteria.epsilon)
                    break;
                // Issue with direct method is r is comuted using distorted x and y instead of undistorted x and y.
                double r2 = x*x + y*y; 
                double icdist = (1 + ((d(7)*r2 + d(6))*r2 + d(5))*r2)/(1 + ((d(4)*r2 + d(1))*r2 + d(0))*r2);

                if (icdist < 0)  // test: undistortPoints.regression_14583
                {
                    x = (u - cx)*ifx;
                    y = (v - cy)*ify;
                    break;
                }
                double deltaX = 2*d(2)*x*y + d(3)*(r2 + 2*x*x)+ d(8)*r2+d(9)*r2*r2;
                double deltaY = d(2)*(r2 + 2*y*y) + 2*d(3)*x*y+ d(10)*r2+d(11)*r2*r2;
                x = (x0 - deltaX)*icdist;
                y = (y0 - deltaY)*icdist;

                if(criteria.type & TermCriteria::EPS) // Iterative way till we reach defined low error.
                {
                    double r4, r6, a1, a2, a3, cdist, icdist2;
                    double xd, yd, xd0, yd0;
                    Eigen::Vector3d vecTilt;

                    r2 = x*x + y*y;
                    r4 = r2*r2;
                    r6 = r4*r2;
                    a1 = 2*x*y;
                    a2 = r2 + 2*x*x;
                    a3 = r2 + 2*y*y;
                    cdist = 1 + d(0)*r2 + d(1)*r4 + d(4)*r6;
                    icdist2 = 1./(1 + d(5)*r2 + d(6)*r4 + d(7)*r6);
                    xd0 = x*cdist*icdist2 + d(2)*a1 + d(3)*a2 + d(8)*r2+d(9)*r4;
                    yd0 = y*cdist*icdist2 + d(2)*a3 + d(3)*a1 + d(10)*r2+d(11)*r4;

                    vecTilt = matTilt * Eigen::Vector3d(xd0, yd0, 1);
                    invProj = vecTilt(2) ? 1./vecTilt(2) : 1;
                    xd = invProj * vecTilt(0);
                    yd = invProj * vecTilt(1);

                    double x_proj = xd*fx + cx;
                    double y_proj = yd*fy + cy;

                    error = sqrt( pow(x_proj - u, 2) + pow(y_proj - v, 2) );
                }
            }
        }
        double xx = proj(0,0)*x + proj(0,1)*y + proj(0,2);
        double yy = proj(1,0)*x + proj(1,1)*y + proj(1,2);
        double ww = 1./(proj(2,0)*x + proj(2,1)*y + proj(2,2));
        x = xx*ww;
        y = yy*ww;
        internalDst.push_back(dai::Point2f(x, y));
    }
    dst = internalDst;
}

void icvGetRectangles(Eigen::Matrix3d& cameraMatrix, Eigen::VectorXd& distCoeffs,
                  Eigen::Matrix3d& R,  Eigen::Matrix3d& newCameraMatrix, int width, 
                  int height, Rect& inner, Rect& outer ) {
    // icvGetRectangles  https://sourcegraph.com/github.com/opencv/opencv@a08c98cdfb781af95dcf481c6ac38ad26ff945c3/-/blob/modules/calib3d/src/calibration.cpp?L2515:1-2515:17
    const int N = 9;
    std::vector<dai::Point2f> pts;
    pts.reserve(N*N);
    int x, y, k; 
    
    for (y = k = 0; y < N; y++){
        for (x = 0; x < N; x++){
            pts.push_back(dai::Point2f((float)x * width / (N-1), (float)y * height / (N-1)));
        }
    }
    
    // Normalized undistorted Points
    undistortPoints(pts, pts, cameraMatrix, distCoeffs, R, newCameraMatrix, TermCriteria(cv::TermCriteria::COUNT, 5, 0.01));

    float iX0 = -FLT_MAX, iX1 = FLT_MAX,  iY0 = -FLT_MAX, iY1 = FLT_MAX;
    float oX0 = FLT_MAX,  oX1 = -FLT_MAX, oY0 = FLT_MAX,  oY1 = -FLT_MAX;

    // find the inscribed rectangle.
    // the code will likely not work with extreme rotation matrices (R) (>45%)
    for( y = k = 0; y < N; y++ ){
        for( x = 0; x < N; x++ ){

            dai::Point2f p = pts[k++];
            oX0 = std::min(oX0, p.x);
            oX1 = std::max(oX1, p.x);
            oY0 = std::min(oY0, p.y);
            oY1 = std::max(oY1, p.y);

            if( x == 0 )
                iX0 = std::max(iX0, p.x);
            if( x == N-1 )
                iX1 = std::min(iX1, p.x);
            if( y == 0 )
                iY0 = std::max(iY0, p.y);
            if( y == N-1 )
                iY1 = std::min(iY1, p.y);
        }
    }

    inner = Rect(iX0, iY0, iX1-iX0, iY1-iY0);
    outer = Rect(oX0, oY0, oX1-oX0, oY1-oY0);
    return;
}

Eigen::Matrix3d customOptimalCameraMatrix(Eigen::Matrix3d cameraMatrix, Eigen::VectorXd distCoeffs, int imgWidth, int imgHeight, double alpha, Rect& validRoi){

    Eigen::Matrix3d newCameraMatrix = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Rect inner, outer;
    // Get inscribed and circumscribed rectangles in normalized
    // (independent of camera matrix) coordinates
    icvGetRectangles(cameraMatrix, distCoeffs, R, newCameraMatrix, imgWidth, imgHeight, inner, outer);
    
    // Projection mapping inner rectangle to viewport
    double fx0 = (imgWidth  - 1) / inner.width;
    double fy0 = (imgHeight - 1) / inner.height;
    double cx0 = -fx0 * inner.x;
    double cy0 = -fy0 * inner.y;

    // Projection mapping outer rectangle to viewport
    double fx1 = (imgWidth  - 1) / outer.width;
    double fy1 = (imgHeight - 1) / outer.height;
    double cx1 = -fx1 * outer.x;
    double cy1 = -fy1 * outer.y;

    // Interpolate between the two optimal projections
    newCameraMatrix(0, 0) = fx0*(1 - alpha) + fx1*alpha;
    newCameraMatrix(1, 1) = fy0*(1 - alpha) + fy1*alpha;
    newCameraMatrix(0, 2) = cx0*(1 - alpha) + cx1*alpha;
    newCameraMatrix(1, 2) = cy0*(1 - alpha) + cy1*alpha;

    icvGetRectangles(cameraMatrix, distCoeffs, R, newCameraMatrix, imgWidth, imgHeight, inner, outer);
    Rect r = inner;
    r &= Rect(0, 0, imgWidth, imgHeight);
    validRoi = r;

    return newCameraMatrix;
    // cvConvert(&matM, newCameraMatrix);

}

// additioanlTuning(){
// }

int main(int argc, char** argv) {
    // Create pipeline  
    std::cout << "Start device" << std::endl;

    dai::Pipeline pipeline;

    // Define source and output
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutVideo = pipeline.create<dai::node::XLinkOut>();

    xoutVideo->setStreamName("video");

    // Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
    // camRgb->setVideoSize(1920, 1080);

    xoutVideo->input.setBlocking(false);
    xoutVideo->input.setQueueSize(1);

    // Linking
    camRgb->video.link(xoutVideo->input);

    dai::Device device;
    dai::CalibrationHandler calibData = device.readCalibration();

    device.startPipeline(pipeline);

    std::cout << "Read device" << std::endl;

    std::vector<std::vector<float>> intrinsics;
    int width, height;
    std::tie(intrinsics, width, height) = calibData.getDefaultIntrinsics(dai::CameraBoardSocket::RGB);
    std::vector<float> d = calibData.getDistortionCoefficients(dai::CameraBoardSocket::RGB);
    cv::Size imgSize(width, height);
    double alpha = 0;
    double alphab = 1;

    Eigen::VectorXd dE = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(d.data(), d.size()).cast<double>();
    Eigen::Matrix3d intrinsicsE;

    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            intrinsicsE(i,j) = intrinsics[i][j];
        }
    }

    if (0){
        std::cout << "Printing Original Intrinsics ->" << std::endl;
        for(int i = 0; i < intrinsics.size(); i++) {
            for (int j = 0; j < intrinsics.front().size(); j++) {
                std::cout << intrinsics[i][j] << " " ;
            }
            std::cout << std::endl;
        }

        std::cout << "Printing intrinsics from Eigen for validation ->" << std::endl;
        std::cout << intrinsicsE << std::endl;

        std::cout << "-------------------xxxxxx----------------" << std::endl;

        std::cout << "Printing original DistortionCoefficients ->" << std::endl;
        for (int i = 0; i < d.size(); i++) {
            std::cout << d[i] << " " ; 
        }
        std::cout << std::endl;
        
        std::cout << "Printing Distortion Coeffs from Eigen for validation ->" << std::endl;
        std::cout << dE << std::endl;

        std::cout << "-------------------xxxxxx----------------" << std::endl;
    }

    cv::Mat K = (cv::Mat_<double>(3,3) << intrinsics[0][0], intrinsics[0][1], intrinsics[0][2],
                                          intrinsics[1][0], intrinsics[1][1], intrinsics[1][2],
                                          intrinsics[2][0], intrinsics[2][1], intrinsics[2][2]);

    cv::Rect validRoi;
    Rect roiRectLocal;
    auto newIntrinsicsE = customOptimalCameraMatrix(intrinsicsE, dE, width, height, alpha, roiRectLocal);
    cv::Mat finalKalpha = (cv::Mat_<double>(3,3) << newIntrinsicsE(0, 0), newIntrinsicsE(0, 1), newIntrinsicsE(0, 2),
                                                    newIntrinsicsE(1, 0), newIntrinsicsE(1, 1), newIntrinsicsE(1, 2),
                                                    newIntrinsicsE(2, 0), newIntrinsicsE(2, 1), newIntrinsicsE(2, 2));

    // ---------------- Additional Scaling from https://github.com/opencv/opencv/issues/7240 -------------

    // -------------------------------------------- ~~ END ~~ ----------------------------------------------------

    if (0){
        std::cout << "-------------------xxxxxx----------------" << std::endl;

        std::cout << "newIntrinsicsEigen --> " << newIntrinsicsE << std::endl;

        std::cout << "-------------------xxxxxx----------------" << std::endl;

        std::cout << "validRoi Local --> " << roiRectLocal.x << " " << roiRectLocal.y << " " << roiRectLocal.width << " " << roiRectLocal.height << std::endl;

        std::cout << "width --> " << width << std::endl;
        std::cout << "height --> " << height << std::endl;

        std::cout << "-------------xxxxxx---xxxxxx---xxxxxx---xxxxxx---xxxxxx----------" << std::endl;
    }

    auto video = device.getOutputQueue("video");

    cv::Mat xMapd, yMapd;
    cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
    cv::initUndistortRectifyMap(K, d, R, finalKalpha, imgSize, CV_16SC2, xMapd, yMapd);

    auto newIntrinsicsEx = customOptimalCameraMatrix(intrinsicsE, dE, width, height, alphab, roiRectLocal);
    cv::Mat finalKalphaOne = (cv::Mat_<double>(3,3) << newIntrinsicsEx(0, 0), newIntrinsicsEx(0, 1), newIntrinsicsEx(0, 2),
                                                       newIntrinsicsEx(1, 0), newIntrinsicsEx(1, 1), newIntrinsicsEx(1, 2),
                                                       newIntrinsicsEx(2, 0), newIntrinsicsEx(2, 1), newIntrinsicsEx(2, 2));

    cv::Mat xMapOne, yMapOne;
    cv::initUndistortRectifyMap(K, d, R, finalKalphaOne, imgSize, CV_16SC2, xMapOne, yMapOne);

    while(true) {
        auto videoIn = video->get<dai::ImgFrame>();

        // Get BGR frame from NV12 encoded video frame to show with opencv
        // Visualizing the frame on slower hosts might have overhead
        cv::Mat imgFrame = videoIn->getCvFrame();
        cv::imshow("video", imgFrame);
        
        cv::Mat undistortedImageOne;
        cv::remap(imgFrame, undistortedImageOne, xMapOne, yMapOne, cv::INTER_LINEAR);
        cv::imshow("undistored finalKalphaOne Video", undistortedImageOne);

        cv::Mat undistortedImaged;
        cv::remap(imgFrame, undistortedImaged, xMapd, yMapd, cv::INTER_LINEAR);
        cv::imshow("undistored finalKalpha Video", undistortedImaged);
        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
