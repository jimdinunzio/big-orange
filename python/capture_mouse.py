#! python3
from pynput import mouse

def on_click(x, y, button, pressed):
    if pressed:
        print(f'X: {x} Y: {y}')

print("Press 'q' then Enter to quit.")
listener = mouse.Listener(on_click=on_click)
listener.start()
try:
    while True:
        if input().strip().lower() == 'q':
            print('\nExiting...')
            break
finally:
    listener.stop()
    listener.join()
