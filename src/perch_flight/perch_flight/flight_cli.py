"""Fly the drone by typing commands. Works the same on the Gazebo sim and the real Pixhawk.

Start the sim and the Micro XRCE-DDS Agent first (see README.md), then:
    ros2 run perch_flight flight_cli

Type help at the vespr> prompt for the commands. Ctrl+C or quit lands the drone first.
"""
import readline  # noqa: F401  importing to give input() up-arrow history
import threading

import rclpy
from rclpy.signals import SignalHandlerOptions

from perch_flight.commander import Commander, Refused

HELP = """\
takeoff [alt]                   arm and climb (default 1.5 m)
goto <north> <east> <alt>       fly to a position, relative to the takeoff spot
forward | back <m>              relative to the nose points
left | right <m>
up | down <m>
yaw <deg>                       turn in place, + clockwise, - counterclockwise
speed <m/s>                     cruise speed for the next moves
hold                            stop where you are (or take control back from the pilot)
land
disarm                          stop the motors, once landing (e.g. hanging from a branch)
status
quit                            lands first if flying"""

# word: (forward, right, up) for a move of 1 m
MOVES = {
    'forward': (1, 0, 0),
    'back': (-1, 0, 0),
    'right': (0, 1, 0),
    'left': (0, -1, 0),
    'up': (0, 0, 1),
    'down': (0, 0, -1),
}


def run_command(commander, line):
    """Runs one typed line. Returns the text to print, and whether to quit."""
    words = line.split()
    if not words:
        return '', False
    name, args = words[0].lower(), words[1:]

    try:
        numbers = [float(a) for a in args]
    except ValueError:
        return f'not a number: {" ".join(args)}', False

    def expect(*counts):
        if len(numbers) not in counts:
            raise Refused(f'{name} takes {" or ".join(str(c) for c in counts)} number(s). Type help')

    try:
        if name == 'help':
            return HELP, False
        if name == 'status':
            return commander.status_text(), False
        if name == 'quit':
            return 'quitting' + (' after landing' if not commander.safe_to_exit() else ''), True
        if name == 'takeoff':
            expect(0, 1)
            return commander.takeoff(*numbers), False
        if name == 'goto':
            expect(3)
            return commander.goto(*numbers), False
        if name in MOVES:
            expect(1)
            forward, right, up = (axis * numbers[0] for axis in MOVES[name])
            return commander.move_body(forward, right, up), False
        if name == 'yaw':
            expect(1)
            return commander.yaw_by(numbers[0]), False
        if name == 'speed':
            expect(1)
            return commander.set_speed(numbers[0]), False
        if name == 'hold':
            expect(0)
            return commander.hold(), False
        if name == 'land':
            expect(0)
            return commander.land(), False
        if name == 'disarm':
            expect(0)
            return commander.disarm(), False
        return f'unknown command: {name}. Type help', False
    except Refused as e:
        return f'refused: {e}', False


def repl(commander, quit_event):
    print('Type help for commands.')
    while not quit_event.is_set():
        try:
            line = input('vespr> ')
        except EOFError:   # Ctrl+D
            line = 'quit'
        text, should_quit = run_command(commander, line)
        if text:
            print(text)
        if should_quit:
            quit_event.set()


def land_if_flying(commander):
    try:
        commander.land()
    except Refused:
        pass   # already on the ground, landing, or with the pilot


def main():
    # Handle Ctrl+C ourselves, so the node keeps running long enough to start the landing
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    commander = Commander()
    quit_event = threading.Event()
    threading.Thread(target=repl, args=(commander, quit_event), daemon=True).start()

    landing_requested = False
    try:
        while rclpy.ok():
            try:
                rclpy.spin_once(commander, timeout_sec=0.1)
            except KeyboardInterrupt:
                if landing_requested:
                    break   # second Ctrl+C: leave now, PX4's offboard-loss failsafe takes over
                print('\nCtrl+C: landing, then quitting. Ctrl+C again to quit right away')
                quit_event.set()

            if quit_event.is_set():
                if not landing_requested:
                    land_if_flying(commander)
                    landing_requested = True
                # PX4's land mode finishes the landing without us
                if commander.safe_to_exit():
                    break
    finally:
        commander.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
