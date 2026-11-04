# readcmd(pad): convert controller inputs to a UART command
# main(): open UART, send commands, and display ESP32 replies

import time
import pygame
import serial

# Raspberry Pi UART
PORT = "/dev/serial0"
BAUD_RATE = 115200

# Controller settings
DEADZONE = 0.05
SEND_INTERVAL = 0.05


def readcmd(pad):

    # Left stick Y controls forward throttle.
    # Right stick X controls steering.
    y = max(-1.0, min(1.0, pad.get_axis(1)))
    x = max(-1.0, min(1.0, pad.get_axis(2)))

    # The confirmed left-bumper button index is 6.
    bumper = pad.get_button(6)

    throttle = 0.0

    if y < -DEADZONE:
        throttle = (-DEADZONE - y) / (1.0 - DEADZONE)

    steer = 0.0

    if x > DEADZONE:
        steer = (x - DEADZONE) / (1.0 - DEADZONE)
    elif x < -DEADZONE:
        steer = (x + DEADZONE) / (1.0 - DEADZONE)

    left = throttle
    right = throttle

    if steer > 0.0:
        right = throttle * (1.0 - steer)
    elif steer < 0.0:
        left = throttle * (1.0 + steer)

    left_cmd = round(max(0.0, min(1.0, left)) * 99)
    right_cmd = round(max(0.0, min(1.0, right)) * 99)

    arm_cmd = "O" if bumper else "C"

    return f"{arm_cmd}{left_cmd:02d}{right_cmd:02d}\n"


def main():

    pygame.init()
    pygame.joystick.init()
    pad = None

    try:

        if pygame.joystick.get_count() == 0:
            raise RuntimeError("Connect the Xbox controller to the Pi.")

        pad = pygame.joystick.Joystick(0)
        pad.init()

        with serial.Serial(
            port=PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0,
            write_timeout=0.2,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False
        ) as esp:

            try:

                # Start with both motors stopped and the arm closed.
                esp.write(b"C0000\n")

                while True:

                    events = pygame.event.get()

                    disconnected = any(
                        event.type == pygame.JOYDEVICEREMOVED
                        and event.instance_id == pad.get_instance_id()
                        for event in events
                    )

                    if disconnected:
                        print("Controller disconnected.")
                        break

                    message = readcmd(pad)
                    esp.write(message.encode("ascii"))

                    # Print any acknowledgment received from the ESP32.
                    reply = esp.read(esp.in_waiting)
                    if reply:
                        print(
                            "ESP32:",
                            reply.decode("ascii", errors="replace"),
                            end=""
                        )

                    time.sleep(SEND_INTERVAL)

            finally:

                # Request stop/closed when the bridge exits.
                try:
                    esp.write(b"C0000\n")
                    esp.flush()
                except serial.SerialException as error:
                    print(f"Could not send final stop command: {error}")

    except KeyboardInterrupt:
        print("\nBridge stopped.")

    finally:

        if pad is not None:
            pad.quit()

        pygame.quit()


if __name__ == "__main__":
    main()