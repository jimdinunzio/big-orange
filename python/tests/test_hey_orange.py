from precise_runner import PreciseEngine, PreciseRunner
from time import sleep

def detectedCb():
    print('Hey Mycroft wake words Detected!')

def start(detectedCb, modelFile):
    engine = PreciseEngine('../Scripts/precise-engine.exe', modelFile)
    runner = PreciseRunner(engine, on_activation=detectedCb)
    runner.start()

def main():
    print("Listening for Hey Orange.")
    start(detectedCb, 'models/hey-orange.pb')
    while True:
        sleep(1)

if __name__ == '__main__':
    main()
