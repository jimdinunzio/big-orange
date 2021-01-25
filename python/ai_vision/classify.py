"""
Created on Fri Jan 22 16:03:08 2021

@author: LattePanda
"""

import argparse
import numpy as np
if __name__ == '__main__':
    import common as common
else:
    from . import common as common
import os
import cv2
import collections
import re
import time
import parse
from PIL import Image
#from tflite_runtime.interpreter import Interpreter
#from tflite_runtime.interpreter import load_delegate
import tflite_runtime.interpreter as tflite


def load_labels(path):
    p = re.compile(r'\s*(\d+)(.+)')
    with open(path, 'r', encoding='utf-8') as f:
       lines = (p.match(line).groups() for line in f.readlines())
       return {int(num): text.strip() for num, text in lines}

# def load_labels(filename):
#   with open(filename, 'r') as f:
#     return [line.strip() for line in f.readlines()]

def classify_image(interpreter, image, top_k=10):
  interpreter.invoke()
  output_details = interpreter.get_output_details()[0]
  output = np.squeeze(interpreter.get_tensor(output_details['index']))

  # If the model is quantized (uint8 data), then dequantize the results
  if output_details['dtype'] == np.uint8:
    scale, zero_point = output_details['quantization']
    output = scale * (output - zero_point)

  ordered_indices = output.argsort()[-top_k:][::-1]
  return [(i, output[i]) for i in ordered_indices]

package_dir = os.path.dirname(os.path.realpath(__file__))
default_model_dir = os.path.join(package_dir, 'all_models')
default_camera_idx = 0

Result = collections.namedtuple('Result', ['percent', 'label'])

model_paths = ['inception_v4_299_quant_edgetpu',
               'mobilenet_v2_1.0_224_inat_bird_quant_edgetpu',
               'mobilenet_v2_1.0_224_inat_insect_quant_edgetpu',
               'mobilenet_v2_1.0_224_inat_plant_quant_edgetpu']


label_paths = ['imagenet_labels',
               'inat_bird_labels',
               'inat_insect_labels',
               'inat_plant_labels']

from enum import Enum
class ModelType(Enum):
    """Enumerated type for status of moving actions"""
    def __init__(self, number):
        self._as_parameter__ = number
    # For general objects
    General = 0
    Birds = 1
    Insects = 2
    Plants = 3

    def model_path(self):
        return os.path.join(default_model_dir, model_paths[self.value] + '.tflite')
    
    def label_path(self):
        return os.path.join(default_model_dir, label_paths[self.value] + '.txt')

def classify(model_type=ModelType.General, top_k=1):
    interpreter = common.make_interpreter(model_type.model_path())
    interpreter.allocate_tensors()
    labels = load_labels(model_type.label_path())
    cap = cv2.VideoCapture(0)
    
    if cap.isOpened():
        for i in range(0,15):
            ret, frame = cap.read()
            time.sleep(1/1000)
            if not ret:
                break
        cv2_im_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pil_im = Image.fromarray(cv2_im_rgb)
        common.set_input(interpreter, pil_im)
        
        results = classify_image(interpreter, pil_im, top_k)
        
        for label_id, prob in results:
            cv2.putText(cv2_im_rgb, labels[label_id], (5,35), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,0,0), 2)
            print('%s: %.5f' % (labels[label_id], prob))
        
        cv2.imshow('Classification', cv2_im_rgb)
        cv2.waitKey(50)
        
        def make(obj):
            fs = "{0}({1})"
            parsed = parse.parse(fs, labels[obj[0]])
            if parsed != None and len(parsed.fixed) > 1:
                tLabel = parsed[1]
            else:
                tLabel = labels[obj[0]]
            return Result(
                label = tLabel,
                percent = int(100 * obj[1])
                )
        cap.release()
        return [make(obj) for obj in results]


def main():
  parser = argparse.ArgumentParser(
      formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument(
      '--model', help='File path of .tflite model.', default='inception_v4_299_quant_edgetpu.tflite')
  parser.add_argument(
      '--labels', help='File path of labels file.', default='imagenet_labels.txt')
  parser.add_argument(
      '--top_k', help='Number of classifications to list', type=int, default=1)
  args = parser.parse_args()

  print('Initializing TF Lite interpreter...')
  
  interpreter = common.make_interpreter(os.path.join(default_model_dir,args.model))
  interpreter.allocate_tensors()
  labels = load_labels(os.path.join(default_model_dir, args.labels))
  cap = cv2.VideoCapture(0)
  
  while (True):
    ret, frame = cap.read()
    cv2_im_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    pil_im = Image.fromarray(cv2_im_rgb)
    common.set_input(interpreter, pil_im)
    
    results = classify_image(interpreter, pil_im, args.top_k)
    for label_id, prob in results:
      cv2.putText(frame, labels[label_id], (5,35), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,0,0), 2)
      print('%s: %.5f' % (labels[label_id], prob))

    cv2.imshow('Classification', frame)
    if cv2.waitKey(1) == ord('q'):
      break

  cap.release()
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  main()
