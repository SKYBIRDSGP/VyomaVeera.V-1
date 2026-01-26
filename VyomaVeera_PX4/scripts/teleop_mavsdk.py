import asyncio
import curses
import mavsdk
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed

# ------------------------------------------------------
# Keyboard teleop
# ------------------------------------------------------
async def teleop(drone):
    stdscr = curses.initscr()
    curses.noecho()
    curses.cbreak()
    stdscr.keypad(True)

    stdscr.addstr(0, 0, "Teleop Active: WASD for movement, I/K up/down, F/G yaw, SPACE arm/disarm, Q quit")

    vx = vy = vz = yaw = 0

    try:
        while True:
            key = stdscr.getch()

            if key == ord('q'):
                break

            elif key == ord(' '):
                # Toggle arm/disarm
                async for state in drone.telemetry.armed():
                    armed = state
                    break

                if not armed:
                    print("Arming…")
                    await drone.action.arm()
                else:
                    print("Disarming…")
                    await drone.action.disarm()

            elif key == ord('w'):
                vx = 1.0
            elif key == ord('s'):
                vx = -1.0
            else:
                vx = 0.0

            if key == ord('a'):
                vy = -1.0
            elif key == ord('d'):
                vy = 1.0
            else:
                vy = 0.0

            if key == ord('i'):
                vz = -0.5    # up in NED
            elif key == ord('k'):
                vz = 0.5     # down
            else:
                vz = 0.0

            if key == ord('f'):
                yaw = -30.0
            elif key == ord('g'):
                yaw = 30.0
            else:
                yaw = 0.0

            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(vx, vy, vz, yaw)
            )

    finally:
        curses.nocbreak()
        stdscr.keypad(False)
        curses.echo()
        curses.endwin()


# ------------------------------------------------------
# Main
# ------------------------------------------------------
async def main():
    print("-- Connecting")
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    print("-- Waiting for drone")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected!")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Sending initial setpoints")
    for i in range(20):
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        )
        await asyncio.sleep(0.05)

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except:
        print("Offboard start failed! Exiting…")
        return

    print("-- Teleoperation Active")
    await teleop(drone)

    print("-- Stopping offboard")
    await drone.offboard.stop()

    print("-- Landing")
    await drone.action.land()


if __name__ == "__main__":
    asyncio.run(main())
