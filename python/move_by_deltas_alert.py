import pygame
import math
from typing import Dict, List

def post_alert(points):
    pygame.init()
    width, height = 500, 500
    screen = pygame.display.set_mode((width, height), pygame.NOFRAME)
    pygame.display.set_caption("Line Plotter")
    # Make window always on top (Windows only)
    try:
        import ctypes
        hwnd = pygame.display.get_wm_info()["window"]
        # Force window to foreground and topmost
        ctypes.windll.user32.SetWindowPos(hwnd, -1, 0, 0, 0, 0, 0x0001 | 0x0002)
        ctypes.windll.user32.ShowWindow(hwnd, 5)  # SW_SHOW
        ctypes.windll.user32.SetForegroundWindow(hwnd)
        ctypes.windll.user32.BringWindowToTop(hwnd)
        # Try to force focus by sending ALT key event
        ctypes.windll.user32.keybd_event(0x12, 0, 0, 0)  # VK_MENU (ALT key down)
        ctypes.windll.user32.keybd_event(0x12, 0, 2, 0)  # VK_MENU (ALT key up)
        ctypes.windll.user32.SetForegroundWindow(hwnd)
        ctypes.windll.user32.BringWindowToTop(hwnd)
    except Exception:
        pass

    font = pygame.font.SysFont(None, 32)
    header_font = pygame.font.SysFont(None, 28)
    number_font = pygame.font.SysFont(None, 24)

    # Calculate offset so first point is always centered
    x0, y0 = points[0]
    def to_screen_coords(x, y):
        return (width // 2 + int(x - x0), height // 2 - int(y - y0))

    screen_points = [to_screen_coords(x, y) for x, y in points]

    # Button definitions
    ok_rect = pygame.Rect(width // 2 - 110, height - 60, 80, 40)
    cancel_rect = pygame.Rect(width // 2 + 30, height - 60, 80, 40)

    def draw_arrow(start, end, color=(0, 0, 0), arrow_length=20, arrow_angle=30):
        # Draw main line
        pygame.draw.line(screen, color, start, end, 2)
        # Calculate direction
        dx, dy = end[0] - start[0], end[1] - start[1]
        angle = math.atan2(dy, dx)
        # Arrow tip position
        tip = end
        # Left side of arrow
        left = (
            tip[0] - arrow_length * math.cos(angle - math.radians(arrow_angle)),
            tip[1] - arrow_length * math.sin(angle - math.radians(arrow_angle))
        )
        # Right side of arrow
        right = (
            tip[0] - arrow_length * math.cos(angle + math.radians(arrow_angle)),
            tip[1] - arrow_length * math.sin(angle + math.radians(arrow_angle))
        )
        pygame.draw.line(screen, color, tip, left, 2)
        pygame.draw.line(screen, color, tip, right, 2)

    def draw():
        screen.fill((255, 255, 255))
        # Draw header text
        header_text = header_font.render("Do you approve this AI planned Path?", True, (0, 0, 0))
        screen.blit(header_text, (width // 2 - header_text.get_width() // 2, 20))
        # Draw lines, arrows, and points
        for i in range(len(screen_points) - 1):
            # Draw line
            pygame.draw.line(screen, (0, 0, 255), screen_points[i], screen_points[i + 1], 2)
            # Draw directional arrow in the middle of the line
            mid_x = (screen_points[i][0] + screen_points[i + 1][0]) // 2
            mid_y = (screen_points[i][1] + screen_points[i + 1][1]) // 2
            # Arrow points from i to i+1
            draw_arrow((mid_x, mid_y), (
                mid_x + (screen_points[i + 1][0] - screen_points[i][0]) // 6,
                mid_y + (screen_points[i + 1][1] - screen_points[i][1]) // 6
            ), color=(0, 0, 0), arrow_length=12, arrow_angle=30)
        # Track number of occurrences for each point
        point_occurrences = {}
        for point in screen_points:
            point_occurrences[point] = point_occurrences.get(point, 0) + 1
        
        # Track current occurrence for each point
        current_occurrence = {}
        
        for idx, (x, y) in enumerate(screen_points):
            pygame.draw.circle(screen, (255, 0, 0), (x, y), 5)
            point = (x, y)
            current_occurrence[point] = current_occurrence.get(point, 0) + 1
            
            # Calculate offset and text based on occurrence number
            if point_occurrences[point] > 1:
                # Render current number with a comma for multiples
                if current_occurrence[point] > 1:
                    # Get width of previous numbers with commas
                    total_width = 0
                    for i in range(1, current_occurrence[point]):
                        prev_text = number_font.render(str(i), True, (0, 0, 0))
                        total_width += prev_text.get_width() + 8  # 8 pixels for comma and space
                    
                    # Position after the previous numbers
                    screen.blit(number_font.render(str(idx + 1), True, (0, 0, 0)), 
                              (x + 8 + total_width, y - 8))
                    
                    # Add comma after all except the last occurrence
                    if current_occurrence[point] < point_occurrences[point]:
                        comma_text = number_font.render(",", True, (0, 0, 0))
                        screen.blit(comma_text, (x + 8 + total_width + number_font.render(str(idx + 1), True, (0, 0, 0)).get_width(), y - 8))
                else:
                    # First occurrence
                    screen.blit(number_font.render(str(idx + 1), True, (0, 0, 0)), (x + 8, y - 8))
                    comma_text = number_font.render(",", True, (0, 0, 0))
                    screen.blit(comma_text, (x + 8 + number_font.render(str(idx + 1), True, (0, 0, 0)).get_width(), y - 8))
            else:
                # Single occurrence
                screen.blit(number_font.render(str(idx + 1), True, (0, 0, 0)), (x + 8, y - 8))
        # Draw OK button
        pygame.draw.rect(screen, (0, 200, 0), ok_rect)
        ok_text = font.render("OK", True, (255, 255, 255))
        screen.blit(ok_text, (ok_rect.x + 20, ok_rect.y + 7))
        # Draw Cancel button
        pygame.draw.rect(screen, (200, 0, 0), cancel_rect)
        cancel_text = font.render("Cancel", True, (255, 255, 255))
        screen.blit(cancel_text, (cancel_rect.x + 5, cancel_rect.y + 7))
        pygame.display.flip()

    draw()

    running = True
    result = None
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                result = None
            elif event.type == pygame.ACTIVEEVENT:
                if event.gain == 1 and event.state == 2:
                    draw()
            elif event.type == pygame.WINDOWSHOWN or event.type == pygame.WINDOWEXPOSED:
                draw()
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if ok_rect.collidepoint(event.pos):
                    running = False
                    result = True
                elif cancel_rect.collidepoint(event.pos):
                    running = False
                    result = False

    pygame.quit()
    return result

if __name__ == "__main__":
    res = post_alert([(0.0, 0.0), (-50.0, -86.6), (50.0, -86.6), (0.0,0.0)])
    print("Result:", res)
