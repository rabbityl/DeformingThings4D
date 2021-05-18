import os
import numpy as np
import cv2

# colors mayavi
c_pink = (224. / 255., 75. / 255., 232. / 255.)
c_blue = (0. / 255., 0. / 255., 255. / 255.)

intrin = [519,  300,  519, 250]

s_index, t_index  = 18, 22

def depth_2_pc (depth , intrin ):
    fx, cx, fy, cy = intrin
    height , width = depth.shape
    u = np.arange (width) * np.ones(  [height, width ])
    v = np.arange (height) * np.ones( [width, height])
    v = np.transpose( v )
    X = (u - cx ) * depth / fx
    Y = (v - cy ) * depth / fy
    Z = depth
    return  np.stack ( [X, Y, Z], -1)

"""load source and target depth frame"""
s_depth = os.path.join("./example/depth", '%04d' % s_index + '.png')
s_depth = cv2.imread(s_depth, -1)/1000.
s_mask = s_depth >0

t_depth = os.path.join("./example/depth", '%04d' % t_index + '.png')
t_depth = cv2.imread(t_depth, -1)/1000.
t_mask = t_depth > 0

"""load flow"""
flow = cv2.imread( './example/sflow/%04d' % s_index + "_%04d"%t_index+ '.exr', -1)


sample_mask = s_mask

flow = flow[sample_mask,:]

s_pc = depth_2_pc(s_depth, intrin)
t_pc = depth_2_pc(t_depth, intrin)

s_pc = s_pc[sample_mask, :]
t_pc = t_pc[t_mask, :]

s_pc = s_pc * np.array( [ [1,-1,-1]])
t_pc = t_pc * np.array( [ [1,-1,-1]])
flow = flow * np.array( [ [1,-1,-1]])


choice = np.random.choice(  s_pc.shape[0] , size= 500, replace= False)
s_pc2  = s_pc[choice]
flow = flow[choice]


"""visualize flow using mayavi"""
import mayavi.mlab as mlab
mlab.points3d(s_pc[ :, 0] , s_pc[ :, 1], s_pc[:,  2], scale_factor=0.01 , color=c_blue)
mlab.points3d(t_pc[ :, 0] , t_pc[  :,1], t_pc[  :,2], scale_factor=0.01 , color=c_pink)
mlab.quiver3d(s_pc2[:, 0], s_pc2[ :, 1], s_pc2[ :, 2],
              flow[:, 0] , flow[:, 1] , flow[:, 2],   scale_factor=1)
mlab.show()