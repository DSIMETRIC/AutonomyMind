import unittest
import numpy as np
from autonomymind_runtime.cpu_vision import decode_yolo, iou_xyxy

class YoloPostprocessTests(unittest.TestCase):
    def test_iou(self):
        self.assertAlmostEqual(iou_xyxy([0,0,10,10],[0,0,10,10]),1.0)

    def test_decode_and_nms(self):
        # rows: cx cy w h class0 class1
        output=np.array([[
            [320,320,100,100,0.9,0.1],
            [322,322,100,100,0.8,0.2],
            [100,100,50,50,0.1,0.95],
        ]],dtype=np.float32)
        d=decode_yolo(output,640,640,640,640,0.25,0.45)
        self.assertEqual(len(d),2)
        self.assertEqual({x["class_id"] for x in d},{0,1})

if __name__=="__main__":
    unittest.main()
