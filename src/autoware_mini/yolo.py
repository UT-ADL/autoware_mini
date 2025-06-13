import cv2
import math
import numpy as np
import onnxruntime

CATEGORY_NUM = 4

class YoloModel(object):
    """Class for a traffic light detector YOLO model"""

    def __init__(self,
                 onnx_path,
                 yolo_masks=[(3, 4, 5), (0, 1, 2)],
                 yolo_anchors=[(10, 14), (23, 27), (37, 58), (81, 82), (135, 169),(344, 319)],
                 obj_threshold=0.1,
                 nms_threshold=0.3,
                 yolo_input_resolution=(608, 608)):
        
        """
        :param onnx_path: path of the onnx yolo model
        :param yolo_masks: a list of 3 three-dimensional tuples for the YOLO masks
        :param yolo_anchors: a list of 9 two-dimensional tuples for the YOLO anchors
        :param object_threshold: threshold for object coverage, float value between 0 and 1
        :param nms_threshold: threshold for non-max suppression algorithm, float value between 0 and 1
        :param input_resolution_yolo: two-dimensional tuple with the target network's (spatial) input resolution in HW order
        """
        self.yolo_model = onnxruntime.InferenceSession(onnx_path, providers=['CUDAExecutionProvider'])

        # Yolo model warm-up
        input_shape = self.yolo_model.get_inputs()[0].shape
        dummy_input = np.random.rand(*input_shape).astype(np.float32)
        self.yolo_model.run(None, {'000_net': dummy_input})

        self.masks = yolo_masks
        self.anchors = yolo_anchors
        self.object_threshold = obj_threshold
        self.nms_threshold = nms_threshold
        self.input_resolution_yolo = yolo_input_resolution

    def predict(self, image):
        """Predicts traffic light bounding boxes, classes and scores based on a given image

        :param image: given image
        :return: a tuple of bounding boxes, classes and scores
        """
        # preprocess image to correct format for YOLO 
        preprocessed_image = self.preprocess_image(image)

        # make a prediction
        yolo_outputs = self.yolo_model.run(None, {'000_net': preprocessed_image})

        # postprocess YOLO input
        yolo_output_shapes = [(1,27,19,19), (1,27,38,38)] #shapes for tiny yolov3
        yolo_outputs = [output.reshape(shape) for output, shape in zip(yolo_outputs, yolo_output_shapes)]

        boxes, classes, scores = self.postprocess(yolo_outputs)

        if len(boxes) > 0:
            rois = self._convert_and_scale_boxes(boxes, image.shape[:2])
            return rois, classes, scores
        else:
            return boxes, classes, scores

    def preprocess_image(self, img):
        """Converts image to a suitable format for YOLO model

        :param img: input image
        :return: preprocessed image
        """
        # Resize to match YOLO input dimensions
        out_img = cv2.resize(img, self.input_resolution_yolo, interpolation=cv2.INTER_LINEAR)
        # Normalize to [0,1]
        out_img = out_img.astype(np.float32) / 255.0
        # HWC to CHW
        out_img = np.transpose(out_img,[2,0,1])
        # CHW to NCHW
        out_img = np.expand_dims(out_img,axis = 0)
        # Convert the image to row-major order, also known as "C order":
        out_img = np.array(out_img, dtype = np.float32, order = 'C')

        return out_img
    
    def postprocess(self, outputs):
        """Take the YOLOv3 outputs generated from a TensorRT forward pass, post-process them
        and return a list of bounding boxes for detected object together with their category
        and their confidences in separate lists.

        :param outputs: outputs from a TensorRT engine in NCHW format
        """
        outputs_reshaped = list()
        for output in outputs:
            outputs_reshaped.append(self._reshape_output(output))

        boxes, categories, confidences = self._process_yolo_output(
            outputs_reshaped, self.input_resolution_yolo)

        return boxes, categories, confidences

    def _reshape_output(self, output):
        """Reshape a TensorRT output from NCHW to NHWC format (with expected C=255),
        and then return it in (height,width,3,85) dimensionality after further reshaping.

        :param output: an output from a TensorRT engine after inference
        """
        output = np.transpose(output, [0, 2, 3, 1])
        _, height, width, _ = output.shape

        # There are CATEGORY_NUM=80 object categories:
        return np.reshape(output, (height, width, 3, 4 + 1 + CATEGORY_NUM))

    def _process_yolo_output(self, outputs_reshaped, resolution_raw):
        """Take in a list of three reshaped YOLO outputs in (height,width,3,85) shape and return
        return a list of bounding boxes for detected object together with their category and their
        confidences in separate lists.

        :param outputs_reshaped: list of three reshaped YOLO outputs as NumPy arrays
        with shape (height,width,3,85)
        :param resolution_raw: the original spatial resolution from the input PIL image in WH order
        """

        # E.g. in YOLOv3-608, there are three output tensors, which we associate with their
        # respective masks. Then we iterate through all output-mask pairs and generate candidates
        # for bounding boxes, their corresponding category predictions and their confidences:
        boxes, categories, confidences = list(), list(), list()
        for output, mask in zip(outputs_reshaped, self.masks):
            box, category, confidence = self._process_feats(output, mask)
            box, category, confidence = self._filter_boxes(box, category, confidence)
            boxes.append(box)
            categories.append(category)
            confidences.append(confidence)

        boxes = np.concatenate(boxes)
        categories = np.concatenate(categories)
        confidences = np.concatenate(confidences)

        # Scale boxes back to original image shape:
        width, height = resolution_raw
        image_dims = [width, height, width, height]
        boxes = boxes * image_dims

        # Using the candidates from the previous (loop) step, we apply the non-max suppression
        # algorithm that clusters adjacent bounding boxes to a single bounding box:
        nms_boxes, nms_categories, nscores = list(), list(), list()
        for category in set(categories):
            idxs = np.where(categories == category)
            box = boxes[idxs]
            category = categories[idxs]
            confidence = confidences[idxs]

            keep = self._nms_boxes(box, confidence)

            nms_boxes.append(box[keep])
            nms_categories.append(category[keep])
            nscores.append(confidence[keep])

        if not nms_categories and not nscores:
            return [], [], []

        boxes = np.concatenate(nms_boxes)
        categories = np.concatenate(nms_categories)
        confidences = np.concatenate(nscores)

        return boxes, categories, confidences

    def _process_feats(self, output_reshaped, mask):
        """Take in a reshaped YOLO output in height,width,3,85 format together with its
        corresponding YOLO mask and return the detected bounding boxes, the confidence,
        and the class probability in each cell/pixel.

        :param output_reshaped: reshaped YOLO output as NumPy arrays with shape (height,width,3,85)
        :param mask: 2-dimensional tuple with mask specification for this output
        """

        # Two in-line functions required for calculating the bounding box
        # descriptors:
        def sigmoid(value):
            """Return the sigmoid of the input."""
            return 1.0 / (1.0 + np.exp(-value))

        grid_h, grid_w, _, _ = output_reshaped.shape

        anchors = [self.anchors[i] for i in mask]

        # Reshape to N, height, width, num_anchors, box_params:
        anchors_tensor = np.reshape(anchors, [1, 1, len(anchors), 2])
        box_xy = sigmoid(output_reshaped[..., :2])
        box_wh = np.exp(output_reshaped[..., 2:4]) * anchors_tensor
        box_confidence = sigmoid(output_reshaped[..., 4])

        box_confidence = np.expand_dims(box_confidence, axis=-1)
        box_class_probs = sigmoid(output_reshaped[..., 5:])

        col = np.tile(np.arange(0, grid_w), grid_w).reshape(-1, grid_w)
        row = np.tile(np.arange(0, grid_h).reshape(-1, 1), grid_h)

        col = col.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
        row = row.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
        grid = np.concatenate((col, row), axis=-1)

        box_xy += grid
        box_xy /= (grid_w, grid_h)
        box_wh /= self.input_resolution_yolo
        box_xy -= (box_wh / 2.)
        boxes = np.concatenate((box_xy, box_wh), axis=-1)

        # boxes: centroids, box_confidence: confidence level, box_class_probs:
        # class confidence
        return boxes, box_confidence, box_class_probs

    def _filter_boxes(self, boxes, box_confidences, box_class_probs):
        """Take in the unfiltered bounding box descriptors and discard each cell
        whose score is lower than the object threshold set during class initialization.

        :param boxes: bounding box coordinates with shape (height,width,3,4); 4 for
        x,y,height,width coordinates of the boxes
        :param box_confidences: bounding box confidences with shape (height,width,3,1); 1 for as
        confidence scalar per element
        :param box_class_probs: class probabilities with shape (height,width,3,CATEGORY_NUM)

        """
        box_scores = box_confidences * box_class_probs
        box_classes = np.argmax(box_scores, axis=-1)
        box_class_scores = np.max(box_scores, axis=-1)
        
        pos = np.where(box_class_scores >= self.object_threshold)

        boxes = boxes[pos]
        classes = box_classes[pos]
        scores = box_class_scores[pos]

        return boxes, classes, scores

    def _nms_boxes(self, boxes, box_confidences):
        """Apply the Non-Maximum Suppression (NMS) algorithm on the bounding boxes with their
        confidence scores and return an array with the indexes of the bounding boxes we want to
        keep (and display later).

        Keyword arguments:
        :param boxes: a NumPy array containing N bounding-box coordinates that survived filtering,
        with shape (N,4); 4 for x,y,height,width coordinates of the boxes
        :param box_confidences: a Numpy array containing the corresponding confidences with shape N
        """
        x_coord = boxes[:, 0]
        y_coord = boxes[:, 1]
        width = boxes[:, 2]
        height = boxes[:, 3]

        areas = width * height
        ordered = box_confidences.argsort()[::-1]

        keep = list()
        while ordered.size > 0:
            # Index of the current element:
            i = ordered[0]
            keep.append(i)
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
            indexes = np.where(iou <= self.nms_threshold)[0]
            ordered = ordered[indexes + 1]

        keep = np.array(keep)
        return keep
    
    def _convert_and_scale_boxes(self, box, original_img_size):
        """Convert yolo output of x_1 y_1 w h to x_1 y_1 x_2 y_2 and scale the boxes based on the original image size

        :param box: a NumPy array containing yolo predicted box
        :param original_img_size: size of the original input image
        """

        x_scale = original_img_size[1] / self.input_resolution_yolo[0]
        y_scale = original_img_size[0] / self.input_resolution_yolo[1]

        x1 = box[:, 0] * x_scale
        y1 = box[:, 1] * y_scale
        x2 = (box[:, 0] + box[:, 2]) * x_scale
        y2 = (box[:, 1] + box[:, 3]) * y_scale

        return np.rint(np.array([x1, y1, x2, y2]).T).astype(int)
