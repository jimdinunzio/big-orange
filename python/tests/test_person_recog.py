from threading import Thread
import facial_recognize as fr

f = fr.FacialRecognize()
t = Thread(target=f.run)
t.start()



f.detected_name


# f = fr.FacialRecognize(camera=True, add_face=True, name="Jim")