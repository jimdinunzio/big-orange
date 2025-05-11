from tkinter import E
import pygame
from pygame.locals import *
import math
import random
from threading import Lock
from orange_utils import OrangeOpType
import my_sdp_client
import sdp_comm

WIDTH = 1024
HEIGHT = 600

_going = False
_pupil_x_off = 0
_pupil_y_off = 0
_target_pupil_x_off = 0
_target_pupil_y_off = 0
_dims = None
_pupil_lock = Lock()
_text_card_text = ""
_pupil_x_move_rate = 0
_pupil_y_move_rate = 0

CMD_TEXT_BOX_LABEL = "Cmd:"
UI_TITLE_TEXT = "Orange Control"
BATT_TEXT_BOX_LABEL = "Battery:"
IP_ADDR_TEXT_BOX_LABEL = "IP Addr:"
INTERNET_TEXT_BOX_LABEL= "Internet:"
WIFI_INFO_LABEL = "SSID:"
LAST_SPEECH_HEARD_LABEL = "Last heard:"
ENERGY_THRESHOLD_LABEL = "Energy Threshold:"
LAST_SPEECH_SPOKEN_LABEL = "Last spoken:"
EXIT_BUTTON_LABEL = "Exit"
GOOGLE_MODE_BUTTON_LABEL = "Google Speech"
HIDE_BUTTON_LABEL = "Hide"
LOCAL_QUAL_LABEL = "Localization Qual:"
BOARD_TEMP_LABEL = "Board Temp.:"

ORANGE_COLOR = "orangered3"

import socket

# dummy stuff for unit testing 
_dummy_google_mode = True

def dummy_op_request(sdp, opType : OrangeOpType, arg1=None, arg2=None):
    global _dummy_google_mode
    if opType == OrangeOpType.TextCommand:
        return True
    elif opType == OrangeOpType.BatteryPercent:
        return 85
    elif opType == OrangeOpType.LastSpeechHeard:
        return "how are you "
    elif opType == OrangeOpType.LastSpeechSpoken:
        return "I'm feeling good"
    elif opType == OrangeOpType.IpAddress:
        return socket.gethostbyname(socket.gethostname())
    elif opType == OrangeOpType.GoogleSpeech:
        return _dummy_google_mode
    elif opType == OrangeOpType.ToggleGoogleSpeech:
        _dummy_google_mode = not _dummy_google_mode
        return _dummy_google_mode
    elif opType == OrangeOpType.InternetStatus:
        return True
    elif opType == OrangeOpType.BatteryIsCharging:
        return True
    elif opType == OrangeOpType.BoardTemperature:
        return 46
    elif opType == OrangeOpType.LocalizationQuality:
        return 66
    elif opType == OrangeOpType.WifiSsidAndStrength:
        return "My Wifi", "95%"
    elif opType == OrangeOpType.SpeechEnergyThreshold:
        return 300

def update():
    pass

def pupil_x(yaw):
    return 70 * math.sin(math.radians(yaw))

def pupil_y(pitch):
    return 70 * math.sin(math.radians(pitch))

def draw_eyes(screen, pupil_x_off=0, pupil_y_off=0):
    def draw_eye(eye_x, eye_y, pitch, yaw):
        pupil_x = int(eye_x + pupil_x_off)
        pupil_y = int(eye_y + pupil_y_off)

        pygame.draw.circle(screen, (255, 255, 255), (eye_x, eye_y), 150)
        pygame.draw.circle(screen, (0, 0, 100), (pupil_x, pupil_y), 50)
    
    draw_eye(_dims[0] / 3 - 40, _dims[1]/ 2, pupil_x_off, pupil_y_off)
    draw_eye(2*_dims[0] / 3 + 40, _dims[1]/ 2, pupil_x_off, pupil_y_off)

def draw_eyes_old(screen, pupil_angle=0, pupil_offset=0):
#    screen.fill((0, 0, 0))
    pupil_angle = math.radians(pupil_angle)
    def draw_eye(eye_x, eye_y, pupil_angle, pupil_offset):
        #mouse_x, mouse_y = pygame.mouse.get_pos()

        #distance_x = mouse_x - eye_x
        #distance_y = mouse_y - eye_y
        #distance = min(math.sqrt(distance_x**2 + distance_y**2), 70)
        pupil_offset = min(pupil_offset, 70)
        #angle = math.atan2(distance_y, distance_x)

        pupil_x = int(eye_x + (math.cos(pupil_angle) * pupil_offset))
        pupil_y = int(eye_y + (math.sin(pupil_angle) * pupil_offset))

        pygame.draw.circle(screen, (255, 255, 255), (eye_x, eye_y), 150)
        pygame.draw.circle(screen, (0, 0, 100), (pupil_x, pupil_y), 50)

    draw_eye(_dims[0] / 3 - 40, _dims[1]/ 2, pupil_angle, pupil_offset)
    draw_eye(2*_dims[0] / 3 + 40, _dims[1]/ 2, pupil_angle, pupil_offset)

def shutdown():
    global _going
    _going = False

# sets pitch and yaw immediately with no transition
def setPitchYaw(pitch=None, yaw=None):
    global _pupil_x_off, _pupil_y_off, _target_pupil_y_off, _target_pupil_x_off
    with _pupil_lock:
        if pitch is not None:
            _pupil_y_off = pupil_y(pitch)
            _target_pupil_y_off = _pupil_y_off
        if yaw is not None:
            _pupil_x_off = pupil_x(yaw)
            _target_pupil_x_off = _pupil_x_off

def next_blink_time():
    return pygame.time.get_ticks() + 1000 + (random.random()*6000)

def next_control_refresh():
    return pygame.time.get_ticks() + 2000

################################################################
# function to calculate the distance between 2 points (XA YA) and (XB YB)
def distance_A_to_B(XA, YA, XB, YB):
    dist = math.sqrt((XB - XA)**2 + (YB - YA)**2)
    return dist

def setHome():
    setTargetPitchYaw(0,0)

MOVE_PIXELS_PER_FRAME = 75.0

# sets pitch and yaw with transition
def setTargetPitchYaw(targetPitch=None, targetYaw=None):
    global _target_pupil_y_off, _target_pupil_x_off, _pupil_y_move_rate, _pupil_x_move_rate
    with _pupil_lock:
        if targetPitch is not None:
            _target_pupil_y_off = pupil_y(targetPitch)
        else:
            _target_pupil_y_off = _pupil_y_off
        if targetYaw is not None:
            _target_pupil_x_off = pupil_x(targetYaw)
        else:
            _target_pupil_x_off = _pupil_x_off
        
        move_frames = distance_A_to_B(_pupil_x_off, _pupil_y_off, _target_pupil_x_off, _target_pupil_y_off) / MOVE_PIXELS_PER_FRAME
        if move_frames > 0:
            _pupil_y_move_rate = (_target_pupil_y_off - _pupil_y_off) / move_frames
            _pupil_x_move_rate = (_target_pupil_x_off - _pupil_x_off) / move_frames
        else:
            _pupil_y_move_rate = 0
            _pupil_x_move_rate = 0

def setText(text, time=5):
    global _text_card_text, _text_card_text_end_time
    _text_card_text = text
    _text_card_text_end_time = pygame.time.get_ticks() + time * 1000

def start(handle_op_request, connect_sdp=True):
    global _going, _dims, _pupil_x_off, _pupil_y_off, _text, _text_card_text_end_time
    global _pupil_x_move_rate, _pupil_y_move_rate
    
    _going = True

    if connect_sdp:
        sdp = my_sdp_client.MyClient()
        connected = sdp_comm.connectToSdp(sdp) == 0
    else:
        sdp = None
        connected = False

    google_mode = handle_op_request(sdp, OrangeOpType.GoogleSpeech)

    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT), RESIZABLE)
    _dims = screen.get_size()
#    pygame.display.set_caption('Orange Eyes')

       # Create The Backgound
    background = pygame.Surface(screen.get_size())
    background = background.convert()
    background.fill((0, 0, 0))

      # Put Text On The Background, Centered
    # if pygame.font:
    #     font = pygame.font.Font(None, 36)
    #     text = font.render("Orange Eyes", 1, (10, 10, 10))
    #     textpos = text.get_rect(centerx=background.get_width()/2)
    #     background.blit(text, textpos)

    # basic font for user typed
    base_font = pygame.font.Font(None, 32)
    button_font = pygame.font.Font(None, 48)
    title_font = pygame.font.Font(None, 60)

    # title text
    title_text_surface = title_font.render(UI_TITLE_TEXT, True, ORANGE_COLOR)
    title_text_pos = title_text_surface.get_rect(centerx=background.get_width()/2, centery=33)
 
    cmd_text = ''
    
    # create battery % field
    batt_box_rect = pygame.Rect(350, 100, 50, 32)
    batt_box_label_surface = base_font.render(BATT_TEXT_BOX_LABEL, True, ORANGE_COLOR)

    board_temp_box_rect = pygame.Rect(750, 100, 50, 32)
    board_temp_label_surface = base_font.render(BOARD_TEMP_LABEL, True, ORANGE_COLOR)

    # create IP address field
    ip_addr_box_rect = pygame.Rect(350, 150, 100, 32)
    ip_addr_box_label_surface = base_font.render(IP_ADDR_TEXT_BOX_LABEL, True, ORANGE_COLOR)
    
    # create localization quality
    local_qual_box_rect = pygame.Rect(750, 150, 100, 32)
    local_qual_box_label_surface = base_font.render(LOCAL_QUAL_LABEL, True, ORANGE_COLOR)

    # create internet status field
    internet_box_rect = pygame.Rect(350, 200, 100, 32)
    internet_box_label_surface = base_font.render(INTERNET_TEXT_BOX_LABEL, True, ORANGE_COLOR)

    # create wifi info 
    wifi_info_box_rect = pygame.Rect(350, 250, 100, 32)
    wifi_info_box_label_surface = base_font.render(WIFI_INFO_LABEL, True, ORANGE_COLOR)

    # create last heard
    last_heard_box_rect = pygame.Rect(350, 300, 200, 32)
    last_heard_box_label_surface = base_font.render(LAST_SPEECH_HEARD_LABEL, True, ORANGE_COLOR)

    # energy threshold value
    energy_thresh_box_rect = pygame.Rect(750, 200, 50, 32)
    energy_thresh_box_label_surface = base_font.render(ENERGY_THRESHOLD_LABEL, True, ORANGE_COLOR)

    # create last spoken
    last_spoken_box_rect = pygame.Rect(350, 350, 200, 32)
    last_spoken_box_label_surface = base_font.render(LAST_SPEECH_SPOKEN_LABEL, True, ORANGE_COLOR)

    # create cmd box rectangle
    cmd_box_rect = pygame.Rect(350, 400, 300, 32)

    # create exit box button
    exit_box_rect = pygame.Rect(20, 20, 150, 100)
    exit_box_label_surface = button_font.render(EXIT_BUTTON_LABEL, True, ORANGE_COLOR)

    # create hide button
    hide_button_rect = pygame.Rect(_dims[0] - 150 - 20, 20, 150, 100)
    hide_button_label_surface = button_font.render(HIDE_BUTTON_LABEL, True, ORANGE_COLOR)
    
    # create google mode button
    google_mode_button_rect = pygame.Rect(20, 380, 180, 50)
    google_mode_button_label_surface = base_font.render(GOOGLE_MODE_BUTTON_LABEL, True, ORANGE_COLOR)
    
    # color_active stores color(lightskyblue3) which
    # gets active when input box is clicked by user
    color_active = pygame.Color('lightskyblue3')
    
    # color_passive store color(chartreuse4) which is
    # color of input box.
    color_passive = pygame.Color('chartreuse4')
    cmd_box_color = color_passive

    button_off_color = pygame.Color('black')
    button_on_color = pygame.Color('chartreuse4')

    google_mode_color = button_on_color if google_mode else button_off_color

    cmd_box_active = False
    cmd_box_label_surface = base_font.render(CMD_TEXT_BOX_LABEL, True, ORANGE_COLOR)
    
    # Display The Background
    screen.blit(background, (0, 0))
    pygame.display.flip()

     # Prepare Game Objects
    clock = pygame.time.Clock()
    time = pygame.time
    time_to_blink = next_blink_time()
    time_to_refresh_control = 0

    _text_card_text = ""
    if pygame.font:
        text_card_font = pygame.font.Font(None, 255)
  
    draw_eyes_enabled = True
    draw_ui_enabled = False

    # Main Loop
    try:
        while _going:
            clock.tick(16)

            # Handle Input Events
            for event in pygame.event.get():
                if event.type == QUIT:
                    going = False
                elif event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                        draw_eyes_enabled = True
                        draw_ui_enabled = False
                    elif event.key == pygame.K_BACKSPACE:
                        # get text input from 0 to -1 i.e. end.
                        if cmd_box_active:
                            cmd_text = cmd_text[:-1]
                    # Unicode standard is used for string
                    # formation
                    else:
                        if cmd_box_active:
                            if event.key == pygame.K_RETURN:
                                handle_op_request(sdp, OrangeOpType.TextCommand, cmd_text)
                                cmd_text = ""
                            else:
                                cmd_text += event.unicode
                elif event.type == pygame.ACTIVEEVENT:
                    if event.state == 1:
                        if event.gain == 1:
                            try:
                                screen = pygame.display.set_mode((WIDTH,HEIGHT), RESIZABLE)
                            except:
                                None
                elif event.type == MOUSEBUTTONDOWN:
                    if draw_eyes_enabled:
                        draw_eyes_enabled = False
                        draw_ui_enabled = True
                    elif draw_ui_enabled:
                        if cmd_box_rect.collidepoint(event.pos):
                            cmd_box_active = True
                        else:
                            cmd_box_active = False                        
                        if exit_box_rect.collidepoint(event.pos):
                            draw_ui_enabled = False
                            draw_eyes_enabled = True
                        elif hide_button_rect.collidepoint(event.pos):
                            try:
                                screen = pygame.display.set_mode((WIDTH,HEIGHT-20), RESIZABLE)
                            except:
                                None
                        elif google_mode_button_rect.collidepoint(event.pos):
                            google_mode = handle_op_request(sdp, OrangeOpType.ToggleGoogleSpeech)
                            google_mode_color = button_on_color if google_mode else button_off_color

            #allsprites.update()

            # Draw Everything
            screen.blit(background, (0, 0))
            #allsprites.draw(screen)

            if abs(_target_pupil_x_off - _pupil_x_off) >= abs(_pupil_x_move_rate):
                _pupil_x_off += _pupil_x_move_rate
            else:
                _pupil_x_off = _target_pupil_x_off
                _pupil_x_move_rate = 0

            if abs(_target_pupil_y_off - _pupil_y_off) >= abs(_pupil_y_move_rate):
                _pupil_y_off += _pupil_y_move_rate
            else:
                _pupil_y_off = _target_pupil_y_off
                _pupil_y_move_rate = 0

            if len(_text_card_text) > 0:
                if text_card_font:
                    text = text_card_font.render(_text_card_text, 1, (255, 255, 255))
                    textpos = text.get_rect(centerx=background.get_width()/2, centery=background.get_height()/2)
                    screen.blit(text, textpos)
                    if pygame.time.get_ticks() > _text_card_text_end_time:
                        _text_card_text = ""
            elif draw_eyes_enabled:
                draw_eyes(screen, _pupil_x_off, _pupil_y_off)
                #blink but not when moving the pupils
                if time.get_ticks() > time_to_blink and _pupil_y_move_rate == 0 and _pupil_x_move_rate == 0:
                    pygame.draw.rect(screen, (0,0,0), (0, 0, _dims[0], _dims[1]/2))
                    pygame.display.flip()
                    time.wait(40)
                    pygame.draw.rect(screen, (0,0,0), (0, _dims[1]/2, _dims[0], _dims[1] / 2))
                    pygame.display.flip()
                    time.wait(300)
                    time_to_blink = next_blink_time()
            elif draw_ui_enabled:
                if cmd_box_active:
                    cmd_box_color = color_active
                else:
                    cmd_box_color = color_passive

                if time.get_ticks() > time_to_refresh_control:
                    battery_str = str(handle_op_request(sdp,OrangeOpType.BatteryPercent)) + "%"
                    if handle_op_request(sdp, OrangeOpType.BatteryIsCharging):
                        battery_str += " [Charging]"
                    last_speech_heard = handle_op_request(sdp, OrangeOpType.LastSpeechHeard)
                    last_speech_spoken = handle_op_request(sdp, OrangeOpType.LastSpeechSpoken)
                    ip_address = handle_op_request(sdp, OrangeOpType.IpAddress)
                    internet_status = handle_op_request(sdp, OrangeOpType.InternetStatus)
                    board_temp_str = str(handle_op_request(sdp, OrangeOpType.BoardTemperature)) + "C"
                    local_qual_str = str(handle_op_request(sdp, OrangeOpType.LocalizationQuality)) + "%"
                    ssid, wifi_strength = handle_op_request(sdp, OrangeOpType.WifiSsidAndStrength)
                    speech_energy_threshold = str(handle_op_request(sdp, OrangeOpType.SpeechEnergyThreshold))
                    time_to_refresh_control = next_control_refresh()

                # draw title 
                screen.blit(title_text_surface, title_text_pos)

                # draw exit button
                pygame.draw.rect(screen, color_passive, exit_box_rect, 8, 12)
                exit_text_pos = exit_box_label_surface.get_rect(centerx=(exit_box_rect.x + exit_box_rect.width/2), 
                                                                centery=(exit_box_rect.y + exit_box_rect.height/2))
                screen.blit(exit_box_label_surface, exit_text_pos)
                
                # draw hide button
                pygame.draw.rect(screen, color_passive, hide_button_rect, 8, 12)
                hide_button_text_pos = hide_button_label_surface.get_rect(centerx=(hide_button_rect.x + hide_button_rect.width/2), 
                                                                          centery=(hide_button_rect.y + hide_button_rect.height/2))
                screen.blit(hide_button_label_surface, hide_button_text_pos)
                
                # draw google mode button
                pygame.draw.rect(screen, google_mode_color, google_mode_button_rect)
                google_mode_label_pos = google_mode_button_label_surface.get_rect(centerx=(google_mode_button_rect.x + google_mode_button_rect.width/2),
                                                          centery=(google_mode_button_rect.y + google_mode_button_rect.height/2))
                screen.blit(google_mode_button_label_surface, google_mode_label_pos)
                
                # draw batt %
                batt_text_surface = base_font.render(battery_str, True, "lightgrey")
                screen.blit(batt_text_surface, (batt_box_rect.x+10, batt_box_rect.y+5))
                # draw label
                screen.blit(batt_box_label_surface, (batt_box_rect.x - batt_box_label_surface.get_width() - 5, batt_box_rect.y + 5))
                
                # draw board temp next to batt
                board_temp_text_surface = base_font.render(board_temp_str, True, "lightgray")
                screen.blit(board_temp_text_surface, (board_temp_box_rect.x + 10, board_temp_box_rect.y+5))
                # draw label
                screen.blit(board_temp_label_surface, (board_temp_box_rect.x - board_temp_label_surface.get_width() - 5, board_temp_box_rect.y + 5))

                # draw last speech heard
                last_heard_text_surface = base_font.render(last_speech_heard, True, "lightgray")
                screen.blit(last_heard_text_surface, (last_heard_box_rect.x+10, last_heard_box_rect.y+5))
                # draw label
                screen.blit(last_heard_box_label_surface, (last_heard_box_rect.x - last_heard_box_label_surface.get_width() - 5, last_heard_box_rect.y + 5))

                # draw energy threshold 
                energy_threshold_text_surface = base_font.render(speech_energy_threshold, True, "lightgray")
                screen.blit(energy_threshold_text_surface, (energy_thresh_box_rect.x + 10, energy_thresh_box_rect.y + 5))
                # draw label
                screen.blit(energy_thresh_box_label_surface, (energy_thresh_box_rect.x - energy_thresh_box_label_surface.get_width() - 5, energy_thresh_box_rect.y + 5))

                # draw last speech spoken
                last_spoken_text_surface = base_font.render(last_speech_spoken, True, "lightgray")
                screen.blit(last_spoken_text_surface, (last_spoken_box_rect.x+10, last_spoken_box_rect.y+5))
                # draw label
                screen.blit(last_spoken_box_label_surface, (last_spoken_box_rect.x - last_spoken_box_label_surface.get_width() - 5, last_spoken_box_rect.y + 5))

                # draw ip address
                ip_addr_text_surface = base_font.render(ip_address, True, "lightgray")
                screen.blit(ip_addr_text_surface, (ip_addr_box_rect.x+10, ip_addr_box_rect.y+5))
                # draw label
                screen.blit(ip_addr_box_label_surface, (ip_addr_box_rect.x - ip_addr_box_label_surface.get_width() - 5, ip_addr_box_rect.y + 5))

                # draw internet status
                internet_text_surface = base_font.render("UP" if internet_status else "DOWN", True, "lightgray")
                screen.blit(internet_text_surface, (internet_box_rect.x+10, internet_box_rect.y+5))
                # draw label
                screen.blit(internet_box_label_surface, (internet_box_rect.x - internet_box_label_surface.get_width() - 5, internet_box_rect.y + 5))

                # draw wifi info
                wifi_info_text_surface = base_font.render(ssid + "  (" + wifi_strength + ")", True, "lightgray")
                screen.blit(wifi_info_text_surface, (wifi_info_box_rect.x+10, wifi_info_box_rect.y+5))
                # draw label
                screen.blit(wifi_info_box_label_surface, (wifi_info_box_rect.x - wifi_info_box_label_surface.get_width() - 5, wifi_info_box_rect.y + 5))

                # draw cmd entry label
                screen.blit(cmd_box_label_surface, (cmd_box_rect.x - cmd_box_label_surface.get_width() - 5, cmd_box_rect.y + 5))
                # draw rectangle and argument passed which should
                # be on screen
                pygame.draw.rect(screen, cmd_box_color, cmd_box_rect)            
                text_surface = base_font.render(cmd_text, True, (255, 255, 255))
                # render cmd text at position stated in arguments
                screen.blit(text_surface, (cmd_box_rect.x+5, cmd_box_rect.y+5))
                # set width of textfield so that text cannot get
                # outside of user's text input
                cmd_box_rect.w = max(300, text_surface.get_width()+10)    

                # draw localization quality
                local_text_surface = base_font.render(local_qual_str, True, "lightgray")
                screen.blit(local_text_surface, (local_qual_box_rect.x+10, local_qual_box_rect.y+5)) 
                #draw label
                screen.blit(local_qual_box_label_surface, (local_qual_box_rect.x - local_qual_box_label_surface.get_width() - 5, local_qual_box_rect.y + 5))           

            pygame.display.flip()
    except KeyboardInterrupt:
        None

    if sdp is not None:
        sdp.disconnect()
        sdp.shutdown_server32(kill_timeout=1)
        sdp = None

    pygame.quit()
