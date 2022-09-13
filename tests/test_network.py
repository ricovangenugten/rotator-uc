#!/usr/bin/env python3

import argparse
import socket
import sys,tty,termios

class KeyGrabber:
    KEY_UP = 1
    KEY_DOWN = 2
    KEY_RIGHT = 3
    KEY_LEFT = 4
    KEY_OTHER = 5

    def get_key(self):
        is_special_key = False

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())

            # Read a single char
            first_char = sys.stdin.read(1)
            if first_char == '\x1b':
                # When a special char is encountered, read following 2 chars
                is_special_key = True
                input = sys.stdin.read(2)
            else:
                # Otherwise just use the first char
                input = first_char
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        if   input == '[A':
            output = self.KEY_UP
        elif input == '[B':
            output = self.KEY_DOWN
        elif input == '[C':
            output = self.KEY_RIGHT
        elif input == '[D':
            output = self.KEY_LEFT
        else:
            output = input.lower()

        return (is_special_key, output)


class EasyCommClient:
    def __init__(self, host, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))
        self.sock.settimeout(0.2)

    def send_command(self, command):
        sent = self.sock.send(("%s\n" % command).encode('utf-8'))
        print("Sent: %s" % command)
        try:
            response = self.sock.recv(1024)
            print("Recv: %s" % response.decode())
        except socket.timeout:
           pass


def main():
    parser = argparse.ArgumentParser(description='Test rotator control over network.')
    parser.add_argument('--host', type=str, help='hostname or ip')
    parser.add_argument('--port', type=int, help='port', default=4533)

    args = parser.parse_args()

    ec = EasyCommClient(args.host, args.port)

    kg = KeyGrabber()

    while True:
        (is_special_key, key) = kg.get_key()
        if is_special_key:
            if   key == kg.KEY_UP:
                ec.send_command("MU")

            elif key == kg.KEY_DOWN:
                ec.send_command("MD")

            elif key == kg.KEY_RIGHT:
                ec.send_command("MR")

            elif key == kg.KEY_LEFT:
                ec.send_command("ML")

        else:
            if   key == 'q':
                quit()

            elif key == 's':
                ec.send_command("SA SE")

            elif key == 'a':
                ec.send_command("AZ")

            elif key == 'e':
                ec.send_command("EL")

            elif key == 'v':
                ec.send_command("VE")

            elif key == 'p':
                ec.send_command("p")



if __name__=='__main__':
    main()
