import usb_pixel_ring_v2 as pixel_ring

class SpeakerPixelRing:
    def __init__(self, mic_array=None, prediction_sensitivity=0.5):
        self.pixel_ring = pixel_ring.PixelRing(mic_array.dev) if mic_array is not None else pixel_ring.find()
        self.prediction_sensitivity = prediction_sensitivity
        self.prediction = 0
        self.target_prediction = 0
        self.LIGHT_COUNT = 12
        self.cutoff = round((1.0 - prediction_sensitivity) * self.LIGHT_COUNT)

    def __del__(self):
        self.pixel_ring.close()

    def setSpeak(self):
        self.pixel_ring.speak()

    def setThink(self):
        self.pixel_ring.think()

    def setStartup(self):
        self.setPaletteForSpin()
        self.setSpin()

    def setEndStartup(self):
        self.setPaletteDefault()
        self.setTrace()

    def setTrace(self):
        self.pixel_ring.trace()

    def setSpin(self):
        self.pixel_ring.spin()

    def setPaletteYellow(self):
        self.pixel_ring.set_color_palette(0x005050,0x402000) 

    def setPaletteRed(self):
        self.pixel_ring.set_color_palette(0x005050,0x400000)
        
    def setPaletteDefault(self):
        self.pixel_ring.set_color_palette(0x003000,0x700800)

    def setPaletteForSpin(self):
        self.pixel_ring.set_color_palette(0x700800,0x003000)

    def setPaletteBootDefault(self):
        self.pixel_ring.set_color_palette(0x005050,0x000050)

    def setPrediction(self,conf):
        self.target_prediction = int(round(conf * self.LIGHT_COUNT))
        if self.prediction < self.target_prediction:
            self.prediction += (self.target_prediction - self.prediction) * 0.5
        elif self.prediction > self.target_prediction:
            self.prediction -= (self.prediction - self.target_prediction) * 0.5
         
        n = min(int(self.prediction), self.cutoff)
        m = max(0, int(self.prediction) - self.cutoff)

        self.pixel_ring.customize(n*[0x10,0x0A,0x05,0x00] + m*[0x70,0x08,0x00,0x00] + (self.LIGHT_COUNT-n-m)*[0,0,0,0])

    def setColoredVolume(self, v):
        n = min(v, self.cutoff)
        m = max(0, v - self.cutoff)
        self.pixel_ring.customize(n*[0x10,0x0A,0x05,0x00] + m*[0x70,0x08,0x00,0x00] + (self.LIGHT_COUNT-n-m)*[0,0,0,0])

    def setBlueVolume(self, v):
        n = min(v, self.cutoff)
        m = max(0, v - self.cutoff)
        self.pixel_ring.customize(n*[0x10,0x0A,0x05,0x00] + m*[0x00,0x08,0x70,0x00] + (self.LIGHT_COUNT-n-m)*[0,0,0,0])

    def setRedVolume(self):
        self.pixel_ring.customize(self.LIGHT_COUNT*[0xFF,0x00,0x00,0x00])

    def setVolume(self,v):
        self.pixel_ring.set_volume(v)

    def setOff(self):
        self.pixel_ring.off()
