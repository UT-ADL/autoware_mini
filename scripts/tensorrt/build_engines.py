#!/usr/bin/env python3

"""Build TensorRT engines for all ONNX models in Autoware Mini.

Engines are cached in ~/.ros (the default ROS working directory) so that
ROS nodes can reuse them without rebuilding. Building engines is slow
(minutes per model) but only needs to be done once per model/GPU/TensorRT
version combination.
"""

import os
import argparse
import onnxruntime

AUTOWARE_MINI_DIR = os.path.realpath(os.path.join(os.path.dirname(__file__), '..', '..'))

MODELS = [
    'data/models/traffic_lights/yolov11_tfl.onnx',
    'data/models/traffic_lights/tlr_model.onnx',
    'data/models/sfa/nulyaronar2_02_30.onnx',
    'data/models/traffic_lights/yolov11_tfl_multilabel.onnx'
]

def main():
    parser = argparse.ArgumentParser(description='Build TensorRT engines for ONNX models')
    parser.add_argument('--fp16', action='store_true', help='Build FP16 engines (default)')
    parser.add_argument('--fp32', action='store_true', help='Build FP32 engines')
    args = parser.parse_args()

    # Default to fp16 if neither flag is set
    if not args.fp16 and not args.fp32:
        args.fp16 = True

    precisions = []
    if args.fp32:
        precisions.append(('FP32', False))
    if args.fp16:
        precisions.append(('FP16', True))

    # Build engines in ~/.ros so ROS nodes find the cache
    os.chdir(os.path.expanduser('~/.ros'))

    for model in MODELS:
        path = os.path.join(AUTOWARE_MINI_DIR, model)
        if not os.path.exists(path):
            print(f'Skipping {model} (file not found)')
            continue
        for label, fp16_enable in precisions:
            print(f'Building {label} engine for {model}...')
            onnxruntime.InferenceSession(path, providers=[
                ('TensorrtExecutionProvider', {
                    'trt_engine_cache_enable': True,
                    'trt_fp16_enable': fp16_enable,
                }),
                'CUDAExecutionProvider',
            ])
            print(f'Done: {model} ({label})')

if __name__ == '__main__':
    main()
