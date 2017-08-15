#!/usr/bin/env python
#
#  THE KITTI VISION BENCHMARK SUITE: ROAD BENCHMARK
#
#  File: simpleExample_evalTrainResults.py
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
import getpass
import computeBaseline, evaluateRoad

#########################################################################
# test script to evaluate training data in perspective domain
#########################################################################


# Make sure your training data is located under /home/%username%/data_road/

if __name__ == "__main__":
	user = getpass.getuser()
	trainDir = '/home/' + user + '/data_road/training'
	outputDir_perspective = '/home/' + user + '/data_road/training/results_perspective'
	evaluateRoad.main(outputDir_perspective, trainDir)
