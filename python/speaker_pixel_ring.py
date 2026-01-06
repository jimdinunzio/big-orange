import usb_pixel_ring_v2 as pixel_ring

class SpeakerPixelRing:
    """Pixel ring LED controller with simulation fallback.
    
    If the mic array is in simulation mode or not available, operates in
    simulation mode where all LED operations are no-ops.
    """
    
    def __init__(self, mic_array=None, prediction_sensitivity=0.5):
        self._sim_mode = False
        self.prediction_sensitivity = prediction_sensitivity
        self.prediction = 0
        self.target_prediction = 0
        self.LIGHT_COUNT = 12
        self.cutoff = round((1.0 - prediction_sensitivity) * self.LIGHT_COUNT)
        
        # Check if mic_array is in sim mode or not available
        if mic_array is None or (hasattr(mic_array, 'is_sim_mode') and mic_array.is_sim_mode()):
            print("WARNING: SpeakerPixelRing running in simulation mode.")
            self._sim_mode = True
            self.pixel_ring = None
        else:
            try:
                self.pixel_ring = pixel_ring.PixelRing(mic_array.dev)
            except Exception as e:
                print(f"WARNING: Could not initialize PixelRing: {e}. Running in simulation mode.")
                self._sim_mode = True
                self.pixel_ring = None

    def is_sim_mode(self) -> bool:
        """Check if running in simulation mode."""
        return self._sim_mode

    def __del__(self):
        if not self._sim_mode and self.pixel_ring is not None:
            self.pixel_ring.close()

    def setSpeak(self):
        if self._sim_mode:
            return
        self.pixel_ring.speak()

    def setThink(self):
        if self._sim_mode:
            return
        self.pixel_ring.think()

    def setStartup(self):
        if self._sim_mode:
            return
        self.setPaletteForSpin()
        self.setSpin()

    def setEndStartup(self):
        if self._sim_mode:
            return
        self.setPaletteDefault()
        self.setTrace()

    def setTrace(self):
        if self._sim_mode:
            return
        self.pixel_ring.trace()

    def setSpin(self):
        if self._sim_mode:
            return
        self.pixel_ring.spin()

    def setPaletteYellow(self):
        if self._sim_mode:
            return
        self.pixel_ring.set_color_palette(0x005050,0x402000) 

    def setPaletteRed(self):
        if self._sim_mode:
            return
        self.pixel_ring.set_color_palette(0x005050,0x400000)
        
    def setPaletteDefault(self):
        if self._sim_mode:
            return
        self.pixel_ring.set_color_palette(0x003000,0x700800)

    def setPaletteForSpin(self):
        if self._sim_mode:
            return
        self.pixel_ring.set_color_palette(0x700800,0x003000)

    def setPaletteBootDefault(self):
        if self._sim_mode:
            return
        self.pixel_ring.set_color_palette(0x005050,0x000050)

    def setPrediction(self,conf):
        self.target_prediction = int(round(conf * self.LIGHT_COUNT))
        if self.prediction < self.target_prediction:
            self.prediction += (self.target_prediction - self.prediction) * 0.5
        elif self.prediction > self.target_prediction:
            self.prediction -= (self.prediction - self.target_prediction) * 0.5
         
        if self._sim_mode:
            return
            
        n = min(int(self.prediction), self.cutoff)
        m = max(0, int(self.prediction) - self.cutoff)

        self.pixel_ring.customize(n*[0x10,0x0A,0x05,0x00] + m*[0x70,0x08,0x00,0x00] + (self.LIGHT_COUNT-n-m)*[0,0,0,0])

    def setColoredVolume(self, v):
        if self._sim_mode:
            return
        n = min(v, self.cutoff)
        m = max(0, v - self.cutoff)
        self.pixel_ring.customize(n*[0x10,0x0A,0x05,0x00] + m*[0x70,0x08,0x00,0x00] + (self.LIGHT_COUNT-n-m)*[0,0,0,0])

    def setBlueVolume(self, v):
        if self._sim_mode:
            return
        n = min(v, self.cutoff)
        m = max(0, v - self.cutoff)
        self.pixel_ring.customize(n*[0x10,0x0A,0x05,0x00] + m*[0x00,0x08,0x70,0x00] + (self.LIGHT_COUNT-n-m)*[0,0,0,0])

    def setRedVolume(self):
        if self._sim_mode:
            return
        self.pixel_ring.customize(self.LIGHT_COUNT*[0xFF,0x00,0x00,0x00])

    def setVolume(self,v):
        if self._sim_mode:
            return
        self.pixel_ring.set_volume(v)

    def setOff(self):
        if self._sim_mode:
            return
        self.pixel_ring.off()
