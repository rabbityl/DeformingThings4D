import bpy
import bmesh
import os
import numpy as np
import mathutils
import cv2
import sys
import time
from mathutils import Matrix, Vector, Quaternion, Euler
from mathutils.bvhtree import BVHTree


D=bpy.data
C=bpy.context
pi = 3.14

def opencv_to_blender(T):
    """T: ndarray 4x4
       usecase: cam.matrix_world =  world_to_blender( np.array(cam.matrix_world))
    """
    origin = np.array(((1, 0, 0, 0),
        (0, -1, 0, 0),
        (0, 0, -1, 0),
        (0, 0,  0, 1)))
    return np.matmul(T,origin) #T * origin

def blender_to_opencv(T):
    transform = np.array(((1, 0, 0, 0),
              (0, -1, 0, 0),
              (0, 0, -1, 0),
              (0, 0, 0, 1)))
    return np.matmul(T,transform)#T * transform


def set_camera( bpy_cam,  angle=pi / 3, W=600, H=500):
    """TODO: replace with setting by intrinsics """
    bpy_cam.angle = angle
    bpy_scene = bpy.context.scene
    bpy_scene.render.resolution_x = W
    bpy_scene.render.resolution_y = H

def look_at(obj_camera, point):
    loc_camera = obj_camera.matrix_world.to_translation()
    direction = point - loc_camera
    # point the cameras '-Z' and use its 'Y' as up
    rot_quat = direction.to_track_quat('-Z', 'Y')
    # assume we're using euler rotation
    obj_camera.rotation_euler = rot_quat.to_euler()

def get_calibration_matrix_K_from_blender(camd):
    '''
    refer to: https://blender.stackexchange.com/questions/15102/what-is-blenders-camera-projection-matrix-model
    the code from the above link is wrong, it cause a slight error for fy in 'HORIZONTAL' mode or fx in "VERTICAL" mode.
    We did change to fix this.
    '''
    f_in_mm = camd.lens
    scene = bpy.context.scene
    resolution_x_in_px = scene.render.resolution_x
    resolution_y_in_px = scene.render.resolution_y
    scale = scene.render.resolution_percentage / 100
    sensor_width_in_mm = camd.sensor_width
    sensor_height_in_mm = camd.sensor_height
    pixel_aspect_ratio = scene.render.pixel_aspect_x / scene.render.pixel_aspect_y
    if (camd.sensor_fit == 'VERTICAL'):
        s_v = resolution_y_in_px * scale / sensor_height_in_mm
        s_u = s_v/pixel_aspect_ratio
    else: # 'HORIZONTAL' and 'AUTO'
        pixel_aspect_ratio = scene.render.pixel_aspect_x / scene.render.pixel_aspect_y
        s_u = resolution_x_in_px * scale / sensor_width_in_mm
        s_v = s_u/pixel_aspect_ratio
    alpha_u = f_in_mm * s_u
    alpha_v = f_in_mm * s_v
    u_0 = resolution_x_in_px*scale / 2
    v_0 = resolution_y_in_px*scale / 2
    skew = 0 # only use rectangular pixels
    K = Matrix(
        ((alpha_u, skew,    u_0),
        (    0  ,  alpha_v, v_0),
        (    0  ,    0,      1 )))
    return K


def anime_read( filename):
    """
    filename: path of .anime file
    return:
        nf: number of frames in the animation
        nv: number of vertices in the mesh (mesh topology fixed through frames)
        nt: number of triangle face in the mesh
        vert_data: vertice data of the 1st frame (3D positions in x-y-z-order)
        face_data: riangle face data of the 1st frame
        offset_data: 3D offset data from the 2nd to the last frame
    """
    f = open(filename, 'rb')
    nf = np.fromfile(f, dtype=np.int32, count=1)[0]
    nv = np.fromfile(f, dtype=np.int32, count=1)[0]
    nt = np.fromfile(f, dtype=np.int32, count=1)[0]
    vert_data = np.fromfile(f, dtype=np.float32, count=nv * 3)
    face_data = np.fromfile(f, dtype=np.int32, count=nt * 3)
    offset_data = np.fromfile(f, dtype=np.float32, count=-1)
    '''check data consistency'''
    if len(offset_data) != (nf - 1) * nv * 3:
        raise ("data inconsistent error!", filename)
    vert_data = vert_data.reshape((-1, 3))
    face_data = face_data.reshape((-1, 3))
    offset_data = offset_data.reshape((nf - 1, nv, 3))
    return nf, nv, nt, vert_data, face_data, offset_data

class AnimeRenderer:

    def __init__(self, anime_file, dum_path):
        #####################################################################
        _, _, _, vert_data, face_data, offset_data = \
            anime_read(anime_file)
        offset_data = np.concatenate ( [ np.zeros( (1, offset_data.shape[1], offset_data.shape[2]) ), offset_data], axis=0)
        '''make object mesh'''
        vertices = vert_data.tolist()
        edges = []
        faces = face_data.tolist()
        mesh_data = bpy.data.meshes.new('mesh_data')
        mesh_data.from_pydata(vertices, edges, faces)
        mesh_data.update()
        the_mesh = bpy.data.objects.new('the_mesh', mesh_data)
        the_mesh.data.vertex_colors.new() # init color
        bpy.context.collection.objects.link(the_mesh)
        #####################################################################
        self.the_mesh = the_mesh
        self.offset_data = offset_data
        self.vert_data = vert_data
        #####################################################################
        self.dum_path = dum_path


    def vis_frame(self, fid):
        '''update geometry to a frame (for debug)'''
        src_offset = self.offset_data[fid]
        bm = bmesh.new()
        bm.from_mesh(self.the_mesh.data)
        bm.verts.ensure_lookup_table()
        bm.faces.ensure_lookup_table()
        for i in range(len(bm.verts)):
            bm.verts[i].co = Vector(self.vert_data[i] + src_offset[i])
        bm.to_mesh(self.the_mesh.data)
        bm.free()


    def depthflowgen(self, flow_skip=1, render_sflow=True ):
        num_frame = self.offset_data.shape[0]
        camera = D.objects["Camera"]
        depth_dir = os.path.join( self.dum_path, "depth")
        if not os.path.exists(depth_dir):
            os.makedirs(depth_dir)
        if render_sflow:
            sflow_dir = os.path.join(self.dum_path, "sflow")
            if not os.path.exists(sflow_dir):
                os.makedirs(sflow_dir)

        #####################################################################
        '''prepare rays, (put this inside the for loop if the camera also moves)'''
        K = get_calibration_matrix_K_from_blender(camera.data)
        print(K)
        fx, fy, cx, cy = K[0][0], K[1][1], K[0][2], K[1][2]
        width, height = C.scene.render.resolution_x, C.scene.render.resolution_y
        cam_blender = np.array(camera.matrix_world)
        print (camera.matrix_world, camera.location)
        cam_opencv = blender_to_opencv(cam_blender)
        u, v = np.meshgrid(range(width), range(height))
        u = u.reshape(-1)
        v = v.reshape(-1)
        pix_position = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u)], -1)
        cam_rotation = cam_opencv[:3, :3]
        pix_position = np.matmul(cam_rotation, pix_position.transpose()).transpose()
        ray_direction = pix_position / np.linalg.norm(pix_position, axis=1, keepdims=True)
        ray_origin = cam_opencv[:3, 3:].transpose()

        ####################################################################
        '''visulize ray geometry(for debug)'''
        vis_ray = False
        if vis_ray:
            ray_end = ray_origin + ray_direction
            ray_vert = np.concatenate([ray_origin, ray_end], axis=0)
            ray_edge = [(0, r_end) for r_end in range(1, len(ray_end) + 1)]
            ray_mesh_data = bpy.data.meshes.new("the_raw")
            ray_mesh_data.from_pydata(ray_vert, ray_edge, [])
            ray_mesh_data.update()
            the_ray = bpy.data.objects.new('the_ray', ray_mesh_data)
            # the_mesh.data.vertex_colors.new()  # init color
            bpy.context.collection.objects.link(the_ray)


        ####################################################################
        """dump intrinsics & extrinsics"""
        intrin_path = os.path.join(self.dum_path, "cam_intr.txt")
        extrin_path = os.path.join(self.dum_path, "cam_extr.txt")
        np.savetxt ( intrin_path, np.array(K))
        np.savetxt (extrin_path, cam_opencv)


        #####################################################################
        # time_spent = 0
        # ray_cnt = 0
        for src_frame_id in range  (num_frame):
            print (src_frame_id)
            tgt_frame_id = src_frame_id + flow_skip
            src_offset = self.offset_data[src_frame_id]
            if  tgt_frame_id > num_frame-1 or tgt_frame_id < 0: # video termi
                flow_exist = False
            else :
                flow_exist = True
                tgt_offset = self.offset_data[tgt_frame_id]
                vert_motion = tgt_offset - src_offset  # [N, 3]


            #####################################################################
            '''update geometry'''
            bm = bmesh.new()
            bm.from_mesh(self.the_mesh.data)
            bm.verts.ensure_lookup_table()
            bm.faces.ensure_lookup_table()
            for i in range(len(bm.verts)):
                bm.verts[i].co = Vector(self.vert_data[i] + src_offset[i])
            bm.to_mesh(self.the_mesh.data)
            self.the_mesh.data.update()


            #####################################################################
            """explicitly cast rays to get point cloud and scene flow"""
            # TODO: speedup the code
            # Currently, the code is a bit slower than directly rendering by composition layer of pass_z and pass_uv, (see: https://github.com/lvzhaoyang/RefRESH/tree/master/blender)
            # but since ray_cast return the faceID, this code is more flexible to use, e.g. generating model2frame dense correspondences)
            raycast_mesh = self.the_mesh
            ray_begin_local = raycast_mesh.matrix_world.inverted() @ Vector(ray_origin[0])
            depsgraph=bpy.context.evaluated_depsgraph_get()
            bvhtree = BVHTree.FromObject(raycast_mesh, depsgraph)
            pcl = np.zeros_like(ray_direction)
            sflow = np.zeros_like(ray_direction)
            for i in range(ray_direction.shape[0]):
                # start = time.time()
                # hit, position, norm, faceID = raycast_mesh.ray_cast(ray_begin_local, Vector(ray_direction[i]), distance=60)
                position, norm, faceID, _ =bvhtree.ray_cast(ray_begin_local, Vector(ray_direction[i]), 50)
                # end = time.time()
                if position: # hit a triangle
                    pcl[i]= Matrix(cam_opencv).inverted() @ raycast_mesh.matrix_world @ position
                    if render_sflow and flow_exist:
                        face = bm.faces[faceID]
                        vert_index = [ v.index for v in face.verts]
                        vert_vector = [ v.co for v in face.verts ]
                        weights = np.array( mathutils.interpolate.poly_3d_calc (vert_vector, position) )
                        flow_vector = (vert_motion[vert_index] * weights.reshape([3,1])).sum(axis=0)
                        sflow[i]=flow_vector
            bm.free()


            #####################################################################
            """dump images"""
            depth = pcl[:,2].reshape((height, width))
            depth = (depth*1000).astype(np.uint16) #  resolution 1mm
            depth_path = os.path.join( depth_dir, "%04d"%src_frame_id + ".png")
            cv2.imwrite(depth_path , depth)
            if render_sflow and flow_exist :
                sflow =  np.matmul( np.linalg.inv (cam_opencv[:3, :3]),  sflow.transpose() ).transpose() #rotate to camera coordinate system (opencv)
                sflow = sflow.reshape((height, width, 3)).astype(np.float32)
                flow_path = os.path.join( sflow_dir, "%04d"%src_frame_id +"_%04d"%tgt_frame_id + ".exr")
                cv2.imwrite (flow_path, sflow)



if __name__ == '__main__':


    argv = sys.argv
    argv = argv[argv.index("--") + 1:]  # get all args after "--"
    print(argv)
    anime_file = argv[0]
    dump_path = argv[1]
    flow_skip = int ( argv[2] )

    #####################################################################
    """delete the default cube (which held the material)"""
    bpy.ops.object.select_all(action='DESELECT')
    bpy.data.objects['Cube'].select_set(state=True)
    bpy.ops.object.delete(use_global=False)

    #####################################################################
    """simply setup the camera"""
    H = 500
    W = 600
    bpy_camera = D.objects['Camera']
    bpy_camera.location, look_at_point = Vector ((2,-2,2)), Vector((0,0,1)) # need to compute this for optimal view point
    look_at(bpy_camera, look_at_point)
    set_camera(bpy_camera.data, angle=pi /3, W=W, H=H)
    bpy.context.view_layer.update() #update camera params


    renderer = AnimeRenderer(anime_file, dump_path)
    renderer.depthflowgen(flow_skip=flow_skip)




