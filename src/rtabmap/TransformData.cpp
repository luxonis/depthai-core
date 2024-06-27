#include "depthai/pipeline/datatype/TransformData.hpp"

namespace dai {
TransformData::TransformData(const rtabmap::Transform& transformRTABMap) {
    transform.matrix = {{{transformRTABMap.r11(), transformRTABMap.r12(), transformRTABMap.r13(), transformRTABMap.o14()},
                 {transformRTABMap.r21(), transformRTABMap.r22(), transformRTABMap.r23(), transformRTABMap.o24()},
                 {transformRTABMap.r31(), transformRTABMap.r32(), transformRTABMap.r33(), transformRTABMap.o34()}}};
}
rtabmap::Transform TransformData::getRTABMapTransform() const {
    return rtabmap::Transform(transform.matrix[0][0],
                                          transform.matrix[0][1],
                                          transform.matrix[0][2],
                                          transform.matrix[0][3],
                                          transform.matrix[1][0],
                                          transform.matrix[1][1],
                                          transform.matrix[1][2],
                                          transform.matrix[1][3],
                                          transform.matrix[2][0],
                                          transform.matrix[2][1],
                                          transform.matrix[2][2],
                                          transform.matrix[2][3]);
}
}  // namespace dai