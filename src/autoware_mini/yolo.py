import cv2
import numpy as np
import onnxruntime
import ast

# Maps (color_id, arrow_id) to final class_id for multilabel models
MULTILABEL_CLASS_MAP = np.array([
    [4, 5, 6, 0],     # green:   left / right / straight / unknown
    [7, 8, 9, 1],     # yellow:  left / right / straight / unknown
    [10, 11, 12, 2],  # red:     left / right / straight / unknown
    [3, 3, 3, 3],     # unknown: always class 3
])

class Yolo11Model(object):
    def __init__(self, onnx_path, confidence_threshold=0.25, nms_threshold=0.7, enable_fp16=False):

        """
        :param onnx_path: path of the onnx yolo model
        :param confidence_threshold: threshold for object confidence score, float value between 0 and 1
        :param nms_threshold: threshold for non-max suppression algorithm, float value between 0 and 1
        :param enable_fp16: enable FP16 inference via TensorRT
        """
        self.onnx_path = onnx_path
        self.yolo_model = onnxruntime.InferenceSession(onnx_path, providers=[
            ("TensorrtExecutionProvider", {
                'trt_engine_cache_enable': True,
                'trt_fp16_enable': enable_fp16,
            }),
            ("CUDAExecutionProvider", {'cudnn_conv_algo_search': 'HEURISTIC'}),
        ])

        # Get model metadata
        meta = self.yolo_model.get_modelmeta()
        custom_metadata = meta.custom_metadata_map

        self.input_name = self.yolo_model.get_inputs()[0].name

        ## Get model input shape from metadata
        assert "imgsz" in custom_metadata, f"Error: ONNX model does not contain the key 'imgsz' in metadata. Model path {onnx_path}"
        self.yolo_input_shape = tuple(ast.literal_eval(custom_metadata['imgsz'])[::-1])

        ## Get class name map from metadata
        assert "names" in custom_metadata, f"Error: ONNX model does not contain the key 'names' in metadata. Model path {onnx_path}"
        self.num_classes = len(ast.literal_eval(custom_metadata["names"]))

        # Detect multilabel model: output has 4 bbox + 4 color + 4 arrow = 12 channels
        output_channels = self.yolo_model.get_outputs()[0].shape[1]
        self.multilabel = (output_channels == 12 and self.num_classes == 13)

        # Yolo model warm-up
        input_shape = self.yolo_model.get_inputs()[0].shape
        dummy_input = np.random.rand(*input_shape).astype(np.float32)
        self.yolo_model.run(None, {self.input_name: dummy_input})

        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold

    def predict(self, image):
        """Detects objects from image

        :param image: given image
        :return: a tuple of bounding boxes, classes and scores
        """
        # preprocess image to correct format for YOLO 
        preprocessed_image, ratio, pad = preprocess_image(image, self.yolo_input_shape)

        # make a prediction
        yolo_outputs = self.yolo_model.run(None, {self.input_name: preprocessed_image})

        # postprocess YOLO input
        boxes, classes, scores = self._postprocess_yolo_output(yolo_outputs[0])

        if boxes.size > 0:
            scaled_boxes = convert_and_scale_boxes(boxes, 1/ratio, pad)
            return scaled_boxes, classes, scores
        else:
            return boxes, classes, scores
        
    def _postprocess_yolo_output(self, raw_yolo_output):
        raw_out = np.squeeze(raw_yolo_output)

        n_det = raw_out.shape[1]
        if n_det <= 0:
            return np.empty((0,)), np.empty((0,)), np.empty((0,))

        if self.multilabel:
            # Multilabel model: channels 4-7 are color logits, 8-11 are arrow logits
            color_ids = np.argmax(raw_out[4:8, :], axis=0)
            color_scores = np.take_along_axis(raw_out[4:8, :], color_ids[None, :], axis=0).squeeze()
            arrow_ids = np.argmax(raw_out[8:12, :], axis=0)

            max_conf_scores = color_scores
            filter_mask = max_conf_scores >= self.confidence_threshold

            valid_classes = MULTILABEL_CLASS_MAP[color_ids[filter_mask], arrow_ids[filter_mask]]
        else:
            assert raw_out.shape[0] == 4 + self.num_classes, f"ONNX output not in valid shape. Model shape {raw_out.shape}. Model path {self.onnx_path}"

            # Standard model: channels 4+ are class probabilities
            class_idxs = np.argmax(raw_out[4:, :], axis=0)
            max_conf_scores = np.take_along_axis(raw_out[4:, :], class_idxs[None, :], axis=0).squeeze()

            filter_mask = max_conf_scores >= self.confidence_threshold
            valid_classes = class_idxs[filter_mask]

        valid_confidences = max_conf_scores[filter_mask]
        valid_bboxes = raw_out[:4, filter_mask].T

        # Use Non-Maximum Supression algorithm to select best fitting bounding boxes for each detected object
        keep_idxs = non_maximum_supression_boxes(valid_bboxes, valid_confidences, self.nms_threshold)

        if keep_idxs.size == 0:
            return np.empty((0,)), np.empty((0,)), np.empty((0,))

        return valid_bboxes[keep_idxs], valid_classes[keep_idxs], valid_confidences[keep_idxs]
    

def preprocess_image(image, yolo_input_resolution):
    """Converts image to a suitable format for YOLO model

    :param image: input image
    :param yolo_input_resolution: size of the yolo input
    :letterbox: whether to use letterbox resizing to keep aspect ratio
    """

    img_shape = image.shape[:2] # (height, width)

    # Scale ratio (new / old)
    r = min(yolo_input_resolution[0] / img_shape[1], yolo_input_resolution[1] / img_shape[0])

    # Compute padding
    new_unpad = int(round(img_shape[1] * r)), int(round(img_shape[0] * r)) # (width, height)
    dw, dh = yolo_input_resolution[0] - new_unpad[0], yolo_input_resolution[1] - new_unpad[1]  # width, height deltas

    dw /= 2  # divide padding into left/right
    dh /= 2  # divide padding into top/bottom
    
    # Resize to match YOLO input dimensions
    image = cv2.resize(image, new_unpad, interpolation=cv2.INTER_LINEAR)

    # Add small offset to avoid issues with rounding .5 pixels
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))

    # Pad
    out_img = cv2.copyMakeBorder(image, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))

    # Check that the image is in C-order (row-major order), convert to float32 and normalize to [0,1]
    if not out_img.flags['C_CONTIGUOUS']:
        out_img = np.array(out_img, dtype = np.float32, order = 'C') / 255.0
    else:
        out_img = out_img.astype(np.float32) / 255.0

    # HWC to CHW
    out_img = np.transpose(out_img,[2,0,1])
    # CHW to NCHW
    out_img = np.expand_dims(out_img, axis=0)

    return out_img, r, (dw, dh)

def non_maximum_supression_boxes(boxes, box_confidences, nms_threshold):
    """Apply the Non-Maximum Suppression (NMS) algorithm on the bounding boxes with their
    confidence scores and return an array with the indexes of the bounding boxes we want to
    keep (and display later).

    Keyword arguments:
    :param boxes: a NumPy array containing N bounding-box coordinates that survived filtering,
    with shape (N,4); 4 for x,y,height,width coordinates of the boxes
    :param box_confidences: a Numpy array containing the corresponding confidences with shape N
    :param nms_threshold: IoU threshold, float value between 0 and 1
    """
    x_coord = boxes[:, 0]
    y_coord = boxes[:, 1]
    width = boxes[:, 2]
    height = boxes[:, 3]

    areas = width * height
    ordered = box_confidences.argsort()[::-1]

    keep_idxs = list()
    while ordered.size > 0:
        # Index of the current element:
        i = ordered[0]
        keep_idxs.append(i)
        xx1 = np.maximum(x_coord[i], x_coord[ordered[1:]])
        yy1 = np.maximum(y_coord[i], y_coord[ordered[1:]])
        xx2 = np.minimum(x_coord[i] + width[i], x_coord[ordered[1:]] + width[ordered[1:]])
        yy2 = np.minimum(y_coord[i] + height[i], y_coord[ordered[1:]] + height[ordered[1:]])

        width1 = np.maximum(0.0, xx2 - xx1 + 1)
        height1 = np.maximum(0.0, yy2 - yy1 + 1)
        intersection = width1 * height1
        union = (areas[i] + areas[ordered[1:]] - intersection)

        # Compute the Intersection over Union (IoU) score:
        iou = intersection / union

        # The goal of the NMS algorithm is to reduce the number of adjacent bounding-box
        # candidates to a minimum. In this step, we keep only those elements whose overlap
        # with the current bounding box is lower than the threshold:
        indexes = np.where(iou <= nms_threshold)[0]
        ordered = ordered[indexes + 1]

    keep_idxs = np.array(keep_idxs)
    return keep_idxs

def convert_and_scale_boxes(box, scale, pad):
    """Convert yolo output of x_1 y_1 w h to x_1 y_1 x_2 y_2 and scale the boxes based on the original image size

    :param box: a NumPy array containing yolo predicted box
    :param scale: scale factor to apply to the box coordinates
    :param pad: padding applied to the image during preprocessing
    """
    # Convert from (x, y, width, height) to (x1, y1, x2, y2)
    hw = box[:, 2] / 2
    hh = box[:, 3] / 2
    x1 = box[:, 0] - hw
    y1 = box[:, 1] - hh
    x2 = box[:, 0] + hw
    y2 = box[:, 1] + hh

    # Account for padding
    x_pad, y_pad = int(pad[0]), int(pad[1])
    x1 -= x_pad
    y1 -= y_pad
    x2 -= x_pad
    y2 -= y_pad

    # Scale the coordinates
    x1, x2 = x1*scale, x2*scale
    y1, y2 = y1*scale, y2*scale

    return np.rint(np.array([x1, y1, x2, y2]).T).astype(int)
