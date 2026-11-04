import time
import pygame
from pygame._sdl2 import controller
import serial

PORT = "/dev/cu.usbserial-0001"
BAUD_RATE = 115200
DEADZONE = 0.05

# initialize controller access
pygame.init()
controller.init()

# find a controller supported by SDL's standard button/axis mapping
indices = [
    i for i in range(controller.get_count())
    if controller.is_controller(i)
]

if not indices:
    pygame.quit()
    raise RuntimeError("Connect the Xbox controller before starting.")

pad = controller.Controller(indices[0])

try:
    with serial.Serial(
        PORT,
        BAUD_RATE,
        timeout=0,
        write_timeout=0.1
    ) as esp:

        time.sleep(2)  # allow for reset when the serial port opens

        try:
            while pad.attached():

                pygame.event.pump()
                pygame.event.clear()

                # right stick X controls steering; left stick Y controls throttle
                raw_x = pad.get_axis(pygame.CONTROLLER_AXIS_RIGHTX)
                raw_y = pad.get_axis(pygame.CONTROLLER_AXIS_LEFTY)

                # 1 when the left bumper is held, otherwise 0
                bumper = int(
                    pad.get_button(
                        pygame.CONTROLLER_BUTTON_LEFTSHOULDER
                    )
                )

                # normalize stick axes to approximately -1.0 through +1.0
                x = max(-1.0, min(1.0, raw_x / 32767.0))
                y = max(-1.0, min(1.0, raw_y / 32767.0))

                # left stick up controls forward throttle; no reverse
                throttle = 0.0

                if y < -DEADZONE:
                    throttle = (-DEADZONE - y) / (1.0 - DEADZONE)

                # right stick controls steering
                steer = 0.0

                if x > DEADZONE:
                    steer = (x - DEADZONE) / (1.0 - DEADZONE)
                elif x < -DEADZONE:
                    steer = (x + DEADZONE) / (1.0 - DEADZONE)

                # mix throttle and steering for forward-only motors
                left = throttle
                right = throttle

                if steer > 0.0:
                    # turn right: left motor gets more command
                    right = throttle * (1.0 - steer)
                elif steer < 0.0:
                    # turn left: right motor gets more command
                    left = throttle * (1.0 + steer)

                # convert each motor command to an integer from 0 to 99
                left_command = round(
                    max(0.0, min(1.0, left)) * 99
                )
                right_command = round(
                    max(0.0, min(1.0, right)) * 99
                )

                # bumper held opens the arm; released closes it
                arm_command = "O" if bumper else "C"

                # format: arm + two-digit left + two-digit right + newline
                message = (
                    f"{arm_command}"
                    f"{left_command:02d}"
                    f"{right_command:02d}\n"
                )
                esp.write(message.encode("ascii"))

                # display any response printed by the ESP32
                reply = esp.read(esp.in_waiting)
                if reply:
                    print(reply.decode("ascii", errors="replace"), end="")

                time.sleep(0.05)

        finally:
            # request closed arm and stopped motors on normal exit
            esp.write(b"C0000\n")

except KeyboardInterrupt:
    pass

finally:
    pad.quit()
    pygame.quit()