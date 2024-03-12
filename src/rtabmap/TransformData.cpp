#include "depthai/pipeline/datatype/TransformData.hpp"

namespace dai {
TransformData::TransformData(const rtabmap::Transform& transformRTABMap) {
    transform.data = {{transformRTABMap.r11(), transformRTABMap.r12(), transformRTABMap.r13(), transformRTABMap.o14()},
                 {transformRTABMap.r21(), transformRTABMap.r22(), transformRTABMap.r23(), transformRTABMap.o24()},
                 {transformRTABMap.r31(), transformRTABMap.r32(), transformRTABMap.r33(), transformRTABMap.o34()}};
}
void TransformData::getRTABMapTransform(rtabmap::Transform& transformRTABMap) const {
    transformRTABMap = rtabmap::Transform(transform.data[0][0],
                                          transform.data[0][1],
                                          transform.data[0][2],
                                          transform.data[0][3],
                                          transform.data[1][0],
                                          transform.data[1][1],
                                          transform.data[1][2],
                                          transform.data[1][3],
                                          transform.data[2][0],
                                          transform.data[2][1],
                                          transform.data[2][2],
                                          transform.data[2][3]);
}
}  // namespace dai