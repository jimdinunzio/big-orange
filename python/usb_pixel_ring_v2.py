
import usb.core
import usb.util


class PixelRing:
    TIMEOUT = 8000

    def __init__(self, dev):
        self.dev = dev

    # trace mode, LEDs changing depends on VAD* and DOA*
    def trace(self):
        self.write(0)

    # mono mode, set all RGB LED to a single color, for example Red(0xFF0000), Green(0x00FF00)， Blue(0x0000FF)
    # color: [red, green, blue, 0]
    def mono(self, color):
        self.write(1, [(color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF, 0])
    
    # mono mode convenience function
    def set_color(self, rgb=None, r=0, g=0, b=0):
        if rgb:
            self.mono(rgb)
        else:
            self.write(1, [r, g, b, 0])

    # turn all off
    def off(self):
        self.mono(0)

    # listen mode, similar with trace mode, but not turn LEDs off
    def listen(self, direction=None):
        self.write(2)

    wakeup = listen

    def speak(self):
        self.write(3)

    # think mode
    def think(self):
        self.write(4)

    wait = think

    # spin mode
    def spin(self):
        self.write(5)

    # custom mode, set each LED to its own color
    # data: [r, g, b, 0] * 12
    def show(self, data):
        self.write(6, data)

    customize = show
    
    # set brightness, range: 0x00~0x1F
    def set_brightness(self, brightness):
        self.write(0x20, [brightness])
    
    # set color palette, for example, pixel_ring.set_color_palette(0xff0000, 0x00ff00) together with pixel_ring.think()
    def set_color_palette(self, a, b):
        self.write(0x21, [(a >> 16) & 0xFF, (a >> 8) & 0xFF, a & 0xFF, 0, (b >> 16) & 0xFF, (b >> 8) & 0xFF, b & 0xFF, 0])

    # set center LED: 0 - off, 1 - on, else - depends on VAD
    def set_vad_led(self, state):
        self.write(0x22, [state])

    # show volume, range: 0 - 12
    def set_volume(self, volume):
        self.write(0x23, [volume])

    # set pattern, 0 - Google Home pattern, others - Echo pattern
    def change_pattern(self, pattern):
        self.write(0x24, [pattern])

    def write(self, cmd, data=[0]):
        self.dev.ctrl_transfer(
            usb.util.CTRL_OUT | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, cmd, 0x1C, data, self.TIMEOUT)

    def close(self):
        """
        close the interface
        """
        usb.util.dispose_resources(self.dev)


def find(vid=0x2886, pid=0x0018):
    dev = usb.core.find(idVendor=vid, idProduct=pid)
    if not dev:
        return

    # configuration = dev.get_active_configuration()

    # interface_number = None
    # for interface in configuration:
    #     interface_number = interface.bInterfaceNumber

    #     if dev.is_kernel_driver_active(interface_number):
    #         dev.detach_kernel_driver(interface_number)

    return PixelRing(dev)



if __name__ == '__main__':
    import time

    pixel_ring = find()

    while True:
        try:
            for v in range(0,12):
                pixel_ring.set_volume(v)
                time.sleep(.05)
            time.sleep(1)
            for v in range(11, -1, -1):
                pixel_ring.set_volume(v)
                time.sleep(.05)
            time.sleep(3)            
            # pixel_ring.wakeup(180)
            # time.sleep(3)
            # pixel_ring.listen()
            # time.sleep(3)
            # pixel_ring.think()
            # time.sleep(3)
            # pixel_ring.set_volume(8)
            # time.sleep(3)
            # pixel_ring.off()
            # time.sleep(3)
        except KeyboardInterrupt:
            break

    pixel_ring.off()