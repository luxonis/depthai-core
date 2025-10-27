import pytest
import depthai as dai
import numpy as np

def test_nndata_tensor():
  nndata = dai.NNData()
  
  tensorA = np.random.rand(3,3,3,3)
  tensorB = np.random.randint(2**8, size=(3,3,3,3))
  tensorC = [[1,5],[1,1],[2,3], [1,2]]
  tensorD = [1.1, 2.1, 3.1, 4.1, 5.1]
  tensorE = ["string", "string2"]

  nndata.addTensor("a", tensorA.astype(np.float16))
  nndata.addTensor("b", tensorB)
  nndata.addTensor("c", tensorC)
  nndata.addTensor("d", np.array(tensorD).astype(np.float32))
  nndata.addTensor("dd", tensorD)
  #nndata.addTensor("e", tensorE) # This should fail
  nndata.addTensor("f", tensorB.astype(np.float16)) 
  nndata.addTensor("g", tensorA)

  assert(nndata.getTensorDatatype("a") == dai.TensorInfo.DataType.FP16)
  assert(nndata.getTensorDatatype("b") == dai.TensorInfo.DataType.INT)
  assert(nndata.getTensorDatatype("c") == dai.TensorInfo.DataType.INT)
  assert(nndata.getTensorDatatype("d") == dai.TensorInfo.DataType.FP32)
  assert(nndata.getTensorDatatype("dd") == dai.TensorInfo.DataType.FP32)
  assert(nndata.getTensorDatatype("f") == dai.TensorInfo.DataType.FP16)
  assert(nndata.getTensorDatatype("g") == dai.TensorInfo.DataType.FP32)

  assert(np.allclose(nndata.getTensor("a"), tensorA, atol=0.002))
  assert((nndata.getTensor("b") == tensorB).all())
  assert((nndata.getTensor("c") == tensorC).all())
  assert(np.allclose(nndata.getTensor("d"), tensorD, atol=0.002))

  assert(np.allclose(nndata.getFirstTensor(), tensorA, atol=0.002)) 

if __name__ == '__main__':
  test_nndata_tensor()