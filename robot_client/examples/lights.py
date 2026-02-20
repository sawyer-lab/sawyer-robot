#!/usr/bin/env python3
"""lights.py — Toggle Sawyer lights interactively.

Pick a light by number and press Enter to toggle it on/off.
Press q to quit.
"""

from sawyer_robot import SawyerRobot


def show(lights, states):
    print()
    for i, name in enumerate(lights, 1):
        symbol = "● ON " if states.get(name) else "○ off"
        print(f"  [{i}] {name:<30} {symbol}")
    print()


def main() -> None:
    print("Sawyer Lights — toggle by number, q to quit")
    print("=" * 50)

    with SawyerRobot() as robot:
        lights = sorted(robot.lights.list_all())
        if not lights:
            print("No lights found.")
            return

        # fetch current states
        states = {name: robot.lights.get(name) for name in lights}
        show(lights, states)

        while True:
            try:
                line = input("toggle> ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            if line in ("q", "quit"):
                break

            try:
                idx = int(line) - 1
                if not 0 <= idx < len(lights):
                    raise ValueError
            except ValueError:
                print(f"  enter a number 1–{len(lights)} or q")
                continue

            name = lights[idx]
            new_state = not states[name]
            robot.lights.set(name, new_state)
            states[name] = new_state
            symbol = "● ON" if new_state else "○ off"
            print(f"  → {name} {symbol}")
            show(lights, states)

    print("Done.")


if __name__ == "__main__":
    main()
