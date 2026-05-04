# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Detect 3D objects in lidar point clouds using deep learning
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# general package imports
import numpy as np
import torch
from easydict import EasyDict as edict

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

# model-related
from tools.objdet_models.resnet.models import fpn_resnet
from tools.objdet_models.resnet.utils.evaluation_utils import decode, post_processing

from tools.objdet_models.darknet.models.darknet2pytorch import Darknet as darknet
from tools.objdet_models.darknet.utils.evaluation_utils import post_processing_v2


# load model-related parameters into an edict
def load_configs_model(model_name='darknet', configs=None):

    # init config file, if none has been passed
    if configs is None:
        configs = edict()

    # get parent directory of this file to enable relative paths
    curr_path = os.path.dirname(os.path.realpath(__file__))
    parent_path = configs.model_path = os.path.abspath(os.path.join(curr_path, os.pardir))

    # set parameters according to model type
    if model_name == "darknet":
        configs.model_path = os.path.join(parent_path, 'tools', 'objdet_models', 'darknet')
        configs.pretrained_filename = os.path.join(configs.model_path, 'pretrained', 'complex_yolov4_mse_loss.pth')
        configs.arch = 'darknet'
        configs.batch_size = 4
        configs.cfgfile = os.path.join(configs.model_path, 'config', 'complex_yolov4.cfg')
        configs.conf_thresh = 0.5
        configs.distributed = False
        configs.img_size = 608
        configs.nms_thresh = 0.4
        configs.num_samples = None
        configs.num_workers = 4
        configs.pin_memory = True
        configs.use_giou_loss = False

    elif model_name == 'fpn_resnet':
        ####### ID_S3_EX1-3 START #######
        print("student task ID_S3_EX1-3")

        configs.model_path = os.path.join(parent_path, 'tools', 'objdet_models', 'resnet')
        configs.pretrained_filename = os.path.join(configs.model_path, 'pretrained', 'fpn_resnet_18_epoch_300.pth')
        configs.arch = 'fpn_resnet'
        configs.batch_size = 4
        configs.conf_thresh = 0.5
        configs.distributed = False
        configs.img_size = 608
        configs.nms_thresh = 0.4
        configs.num_samples = None
        configs.num_workers = 4
        configs.pin_memory = True

        # ResNet-specific settings
        configs.num_layers = 18          # ResNet-18 backbone
        configs.K = 50                   # max number of detections per image
        configs.no_cuda = True
        configs.num_classes = 3          # Car, Pedestrian, Cyclist
        configs.num_center_offset = 2
        configs.num_z = 1
        configs.num_dim = 3
        configs.num_direction = 2        # sin/cos encoding of yaw
        configs.heads = {
            'hm_cen':   configs.num_classes,
            'cen_offset': configs.num_center_offset,
            'direction': configs.num_direction,
            'z_coor':   configs.num_z,
            'dim':      configs.num_dim
        }
        configs.down_ratio = 4           # output stride of feature pyramid
        configs.peak_thresh = 0.2

        ####### ID_S3_EX1-3 END #######

    else:
        raise ValueError("Error: Invalid model name")

    # GPU vs. CPU
    configs.no_cuda = True  # if true, cuda is not used
    configs.gpu_idx = 0     # GPU index to use.
    configs.device = torch.device('cpu' if configs.no_cuda else 'cuda:{}'.format(configs.gpu_idx))

    return configs


# load all object-detection parameters into an edict
def load_configs(model_name='fpn_resnet', configs=None):

    # init config file, if none has been passed
    if configs is None:
        configs = edict()

    # birds-eye view (bev) parameters
    configs.lim_x = [0, 50]       # detection range in m
    configs.lim_y = [-25, 25]
    configs.lim_z = [-1, 3]
    configs.lim_r = [0, 1.0]      # reflected lidar intensity
    configs.bev_width = 608       # pixel resolution of bev image
    configs.bev_height = 608
    configs.min_iou = 0.5

    # add model-dependent parameters
    configs = load_configs_model(model_name, configs)

    # visualization parameters
    configs.output_width = 608    # width of result image (height may vary)
    configs.obj_colors = [[0, 255, 255], [0, 0, 255], [255, 0, 0]]  # Pedestrian, Car, Cyclist

    return configs


# create model according to selected model type
def create_model(configs):

    # check for availability of model file
    assert os.path.isfile(configs.pretrained_filename), "No file at {}".format(configs.pretrained_filename)

    # create model depending on architecture name
    if (configs.arch == 'darknet') and (configs.cfgfile is not None):
        print('using darknet')
        model = darknet(cfgfile=configs.cfgfile, use_giou_loss=configs.use_giou_loss)

    elif 'fpn_resnet' in configs.arch:
        ####### ID_S3_EX1-4 START #######
        print("student task ID_S3_EX1-4")
        print('using ResNet architecture with feature pyramid')

        model = fpn_resnet.get_pose_net(
            num_layers=configs.num_layers,
            heads=configs.heads,
            head_conv=64,
            imagenet_pretrained=False
        )

        ####### ID_S3_EX1-4 END #######

    else:
        assert False, 'Undefined model backbone'

    # load model weights
    model.load_state_dict(torch.load(configs.pretrained_filename, map_location='cpu'))
    print('Loaded weights from {}\n'.format(configs.pretrained_filename))

    # set model to evaluation state
    configs.device = torch.device('cpu' if configs.no_cuda else 'cuda:{}'.format(configs.gpu_idx))
    model = model.to(device=configs.device)  # load model to either cpu or gpu
    model.eval()

    return model


# detect trained objects in birds-eye view
def detect_objects(input_bev_maps, model, configs):

    # deactivate autograd engine during test to reduce memory usage and speed up computations
    with torch.no_grad():

        # perform inference
        outputs = model(input_bev_maps)

        # decode model output into target object format
        if 'darknet' in configs.arch:

            # perform post-processing
            output_post = post_processing_v2(outputs, conf_thresh=configs.conf_thresh, nms_thresh=configs.nms_thresh)
            detections = []
            for sample_i in range(len(output_post)):
                if output_post[sample_i] is None:
                    continue
                detection = output_post[sample_i]
                for obj in detection:
                    x, y, w, l, im, re, _, _, _ = obj
                    yaw = np.arctan2(im, re)
                    detections.append([1, x, y, 0.0, 1.50, w, l, yaw])

        elif 'fpn_resnet' in configs.arch:
            ####### ID_S3_EX1-5 START #######
            print("student task ID_S3_EX1-5")

           # decode the heatmap output of the ResNet head into detections
            outputs['hm_cen'] = torch.sigmoid(outputs['hm_cen'])
            outputs['cen_offset'] = torch.sigmoid(outputs['cen_offset'])
            # post_processing returns a list of detections per sample in the batch
            detections = decode(
            outputs['hm_cen'],
            outputs['cen_offset'],
            outputs['direction'],
            outputs['z_coor'],
            outputs['dim'],
            K=configs.K
            )
            # convert tensor to numpy before post_processing
            detections = detections.cpu().numpy()
            detections = post_processing(detections, configs)
            detections = detections[0][1]

            # convert dict to numpy array if needed
            if isinstance(detections, dict):
               detections = np.array(list(detections.values()))

            ####### ID_S3_EX1-5 END #######

    ####### ID_S3_EX2 START #######
    print("student task ID_S3_EX2")
    objects = []

    # pixel-to-metric scale factors for the BEV image
    scale_x = (configs.lim_x[1] - configs.lim_x[0]) / configs.bev_width
    scale_y = (configs.lim_y[1] - configs.lim_y[0]) / configs.bev_height

    ## step 1 : check whether there are any detections
    if detections is None:
        return objects

    ## step 2 : loop over all detections
    for det in detections:
        # det format depends on model:
        # darknet:    [class, x, y, z, h, w, l, yaw]  (already metric)
        # fpn_resnet: [batch_id, class_id, y_bev, x_bev, z, h, w, l, yaw]

        if 'darknet' in configs.arch:
            _id, x, y, z, h, w, l, yaw = det

            ## step 3 : check that the object is within the detection range
            if not (configs.lim_x[0] <= x <= configs.lim_x[1]):
                continue
            if not (configs.lim_y[0] <= y <= configs.lim_y[1]):
                continue

            ## step 4 : append to objects list  [id, x, y, z, h, w, l, yaw]
            objects.append([1, x, y, z, h, w, l, yaw])

        elif 'fpn_resnet' in configs.arch:
            # det = [batch_id, class_id, y_bev_px, x_bev_px, z, h, w, l, yaw]
            _class, y_px, x_px, z, h, w, l, yaw = det

            ## step 3 : convert pixel coordinates to metric (world) coordinates
            x = x_px * scale_x + configs.lim_x[0]
            y = y_px * scale_y + configs.lim_y[0]

            # check detection is within configured range
            if not (configs.lim_x[0] <= x <= configs.lim_x[1]):
                continue
            if not (configs.lim_y[0] <= y <= configs.lim_y[1]):
                continue
            if not (configs.lim_z[0] <= z <= configs.lim_z[1]):
                continue

            ## step 4 : append to objects list  [id, x, y, z, h, w, l, yaw]
            objects.append([1, x, y, z, h, w, l, yaw])

    ####### ID_S3_EX2 END #######

    return objects
