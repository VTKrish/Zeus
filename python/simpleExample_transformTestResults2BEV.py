#!/usr/bin/env python
#
#  THE KITTI VISION BENCHMARK SUITE: ROAD BENCHMARK
#
#  File: simpleExample_transformTestResults2BEV.py
#
#  Copyright (C) 2013
#  Honda Research Institute Europe GmbH
#  Carl-Legien-Str. 30
#  63073 Offenbach/Main
#  Germany
#
#  UNPUBLISHED PROPRIETARY MATERIAL.
#  ALL RIGHTS RESERVED.
#
#  Authors: Tobias Kuehnl <tkuehnl@cor-lab.uni-bielefeld.de>
#           Jannik Fritsch <jannik.fritsch@honda-ri.de>
#

import os, sys
import computeBaseline, transform2BEV

#########################################################################
# test script to process testing data in perspective domain and 
# transform the results to the metric BEV 
#########################################################################

if __name__ == "__main__":
    
    datasetDir = '/home/keenburn2004/data_road'
    outputDir_perspective = '/home/keenburn2004/data_road/testing/test_baseline_perspective'
    outputDir_bev = '/home/keenburn2004/data_road/testing/test_baseline_bev'
    testData_pathToCalib = os.path.join(datasetDir, 'testing/calib')
    print(testData_pathToCalib)space,
    # you need to run this script before submission!
    inputFiles = os.path.join(outputDir_perspective, '*.png')
    print(inputFiles)
    transform2BEV.main(inputFiles, testData_pathToCalib, outputDir_bev)
    # now zip the contents in the directory 'outputDir_bev' and upload
    # the zip file to the KITTI server


    
