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
        key = self.KEY_OTHER

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())

            # Wait for input of escape char, ignore any other input
            while sys.stdin.read(1) != '\x1b':
                pass

            # Escape char found, read the following 2 bytes
            ch = sys.stdin.read(2)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        if   ch=='[A':
            key = self.KEY_UP
        elif ch=='[B':
            key = self.KEY_DOWN
        elif ch=='[C':
            key = self.KEY_RIGHT
        elif ch=='[D':
            key = self.KEY_LEFT

        return key


def main():
    parser = argparse.ArgumentParser(description='Test rotator control over network.')
    parser.add_argument('--host', type=str, help='hostname or ip')
    parser.add_argument('--port', type=int, help='port')

    args = parser.parse_args()

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((args.host, args.port))

    kg = KeyGrabber()

    for i in range(0,20):
        key = kg.get_key()
        if key == kg.KEY_UP:
            print("up")
            sent = s.send(b"MU\n")
        if key == kg.KEY_DOWN:
            print("down")
            sent = s.send(b"MD\n")
        if key == kg.KEY_RIGHT:
            print("right")
            sent = s.send(b"MR\n")
        if key == kg.KEY_LEFT:
            print("left")
            sent = s.send(b"ML\n")


        print("Sent %d bytes" % sent)


if __name__=='__main__':
    main()
