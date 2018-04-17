#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Juergen Sturm, TUM
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of TUM nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior writte n permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# the resulting .ply file can be viewed for example with meshlab
# sudo apt-get install meshlab

"""
This script reads a registered pair of color and depth images and generates a
colored 3D point cloud in the PLY format.
"""

import argparse
import sys
import os
from PIL import Image
import numpy as np
from scipy import misc
import scipy.ndimage

focalLength = 525.0
centerX = 319.5
centerY = 239.5
scalingFactor = 2.0

#def generate_pointcloud(rgb_file,depth_file,ply_file):
def generate_pointcloud():
    """
    Generate a colored point cloud in PLY format from a color and a depth image.
    
    Input:
    rgb_file -- filename of color image
    depth_file -- filename of depth image
    ply_file -- filename of ply file
    
    """
    
    master_dir = "/mnt/lustre/wangjiayun/dataset/vivo3d/vivo_sample/3/"
    rgb = Image.open(master_dir + "rgb.png")
    #depth = Image.open(master_dir + "dep.png").convert('I')
    dep = np.fromfile(master_dir + "dep_proc.raw", dtype=np.uint16).reshape(480, 640)[::-1, :]
    dep = scipy.ndimage.rotate(dep, 90)
    tempmin = 0 #np.amin(np.array(dep.flatten())[dep.flatten() != 0])
    dep = (dep - tempmin)*1.0 / (np.max(dep.flatten() ) - tempmin)
    #dep[dep == dep[0][0]] = 0

    #rgb = rgb.crop((363,1,574,266))
    #depth = depth.crop((363,1,574,266))
    '''if rgb.size != depth.size:
        raise Exception("Color and depth image do not have the same resolution.")
    if rgb.mode != "RGB":
        raise Exception("Color image is not in RGB format")
    if depth.mode != "I":
        raise Exception("Depth image is not in intensity format")'''


    points = []    
    for v in range(rgb.size[1]):
        for u in range(rgb.size[0]):
            color = rgb.getpixel((u,v))
            #Z = depth.getpixel((u,v)) / scalingFactor
            Z = dep[v, u] *1.0/ scalingFactor
            if Z==0: continue
            X = (u - centerX) * Z / focalLength
            Y = (v - centerY) * Z / focalLength
            points.append("%f %f %f %d %d %d 0\n"%(X,Y,Z,color[0],color[1],color[2]))
    file = open(master_dir + "pc.ply","w")
    file.write('''ply
format ascii 1.0
element vertex %d
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
property uchar alpha
end_header
%s
'''%(len(points),"".join(points)))
    file.close()

if __name__ == '__main__':
    """parser = argparse.ArgumentParser(description='''
    This script reads a registered pair of color and depth images and generates a colored 3D point cloud in the
    PLY format. 
    ''')
    parser.add_argument('rgb_file', help='input color image (format: png)')
    parser.add_argument('depth_file', help='input depth image (format: png)')
    parser.add_argument('ply_file', help='output PLY file (format: ply)')
    args = parser.parse_args()"""

    #generate_pointcloud(args.rgb_file,args.depth_file,args.ply_file)
    generate_pointcloud()
    

