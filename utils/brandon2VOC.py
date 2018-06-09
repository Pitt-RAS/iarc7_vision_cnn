#!/usr/bin//env python
from PIL import Image
from collections import deque
import pascal_voc_writer as pvw
import os
import sys
"""
To use run: python brandon2VOC.py yoloV2annotation.txt
Requires annotations and images to be in the same directory with the same names
before the extenstion
"""

#create brandon label
class BrandonLabel():
    def __init__(self,x1,y1,x2,y2,cn):
        #class_number x1_ratio y1_ratio width_ratio height_ratio
        self.class_number = cn
        self.x1 = float(x1)
        self.y1 = float(y1)
        self.x2 = float(x2)
        self.y2 = float(y2)

    def bran2voc(self):
        #VOC
        #<class> <bboxXMin> <bboxYMin> <bboxXMax> <bboxYMax>
        voc = [0,0,0,0]
        voc[0] = self.x1
        voc[1] = self.y1
        voc[2] = self.x2
        voc[3] = self.y2
        return voc

class BrandonFile():
    def __init__(self,fname):
        self.fname  = fname  + ".txt"
        self.imname = fname + ".jpg"
        self.yolo_q = deque()
        self.im = Image.open(self.imname)
        self.width, self.height = self.im.size
        self.yl_list = []

    def readr(self):
        # Read the fucking file and queue all the shit
        f = open(self.fname,"r")
        yl  = 0
        for line in f.readlines():
            #read line
            yolo_params = line.split()
            #append line to queue
            self.yl_list.append(BrandonLabel(yolo_params[0],yolo_params[1],\
                    yolo_params[2],yolo_params[3],yolo_params[4]))
        f.close()

    def writer(self):
        writer = pvw.Writer(self.imname,self.width,self.height)
        while(self.yl_list):
            obj_label = self.yl_list.pop()
            voc = obj_label.bran2voc()
            writer.addObject(obj_label.class_number,voc[0],voc[1],voc[2],voc[3])

        writer.save(fname + ".xml")

fname = sys.argv[1]
fname,ext=os.path.splitext(fname)
yFile = BrandonFile(fname)
yFile.readr()
yFile.writer()

