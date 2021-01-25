import argparse
import numpy as np

import cv2
from PIL import Image
from tflite_runtime.interpreter import Interpreter
from tflite_runtime.interpreter import load_delegate

def load_labels(filename):
  with open(filename, 'r') as f:
    return [line.strip() for line in f.readlines()]

def set_input_tensor(interpreter, image):
  tensor_index = interpreter.get_input_details()[0]['index']
  input_tensor = interpreter.tensor(tensor_index)()[0]
  input_tensor[:, :] = image

def classify_image(interpreter, image, top_k=10):
  set_input_tensor(interpreter, image)
  interpreter.invoke()
  output_details = interpreter.get_output_details()[0]
  output = np.squeeze(interpreter.get_tensor(output_details['index']))

  # If the model is quantized (uint8 data), then dequantize the results
  if output_details['dtype'] == np.uint8:
    scale, zero_point = output_details['quantization']
    output = scale * (output - zero_point)

  ordered_indices = output.argsort()[-top_k:][::-1]
  return [(i, output[i]) for i in ordered_indices]

def main():
  parser = argparse.ArgumentParser(
      formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument(
      '--model', help='File path of .tflite file.', required=True)
  parser.add_argument(
      '--labels', help='File path of labels file.', required=True)
  parser.add_argument(
      '--top_k', help='Number of classifications to list', type=int, default=1)
  args = parser.parse_args()

  print('Initializing TF Lite interpreter...')
  interpreter = Interpreter(
      model_path=args.model,
      experimental_delegates=[load_delegate('libedgetpu.so.1.0')])
  interpreter.allocate_tensors()
  _, height, width, _ = interpreter.get_input_details()[0]['shape']

  cap = cv2.VideoCapture(0)
  labels = load_labels(args.labels)
  while (True):
    ret, frame = cap.read()
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_resized = cv2.resize(frame_rgb, (width, height))
    image = Image.fromarray(frame_resized)
    results = classify_image(interpreter, image, args.top_k)
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
