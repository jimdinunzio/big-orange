from usb_pixel_ring_v2 import PixelRing
import usb.core
import usb.util
import time
 
dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
print(dev)
if dev:
    pixel_ring = PixelRing(dev)
 
    while True:
        try:
            pixel_ring.wakeup(180)
            time.sleep(3)
            pixel_ring.listen()
            time.sleep(3)
            pixel_ring.think()
            time.sleep(3)
            pixel_ring.set_volume(8)
            time.sleep(3)
            pixel_ring.off()
            time.sleep(3)
        except KeyboardInterrupt:
            break
 
    pixel_ring.off()
