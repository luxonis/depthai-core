try:
    import os
    import subprocess
    import pathlib
    current_dir = pathlib.Path(__file__).parent.absolute()
except Exception as e:
    print(e)
    raise e

STATUS_LED_PATH = "/dev/status_led"


def shell_source(script):
    """Get the environment that is created by sourcing the script"""
    pipe = subprocess.Popen(". %s > /dev/null; env" % script, stdout=subprocess.PIPE, shell=True)
    output = pipe.communicate()[0]
    env = dict((line.split("=", 1) for line in output.decode().splitlines()))
    return env


def turn_on_led(brightness, r, g, b):
    """Writes LED values directly in the order expected by the device."""
    try:
        with open(STATUS_LED_PATH, "w") as f:
            f.write(f"{brightness},{r},{g},{b}")
    except Exception:
        pass


def read_led_color_from_conf(key, default=(255, 0, 0, 168)):
    """
    Reads LED color from /etc/agentconfd/conf in ARGB format (0xAARRGGBB)
    and returns a device-ready 4-tuple: (brightness=A, R, G, B)
    """
    conf_path = "/etc/agentconfd/conf"
    try:
        with open(conf_path, "r") as f:
            for line in f:
                if line.startswith(key + "="):
                    val = line.strip().split("=")[1]
                    if val.startswith("0x"):
                        hexval = val[2:]
                        if len(hexval) == 8:
                            a = int(hexval[0:2], 16)  # brightness
                            r = int(hexval[2:4], 16)
                            g = int(hexval[4:6], 16)
                            b = int(hexval[6:8], 16)
                            return (a, r, g, b)
    except Exception:
        pass
    return default


def read_current_led():
    """
    Reads the current LED value from /dev/status_led.

    Returns:
        tuple[int, ...] | None
        - (brightness, R, G, B) or (R, G, B) depending on what the driver returns
        - None if it cannot be read / parsed
    """
    try:
        fd = os.open(STATUS_LED_PATH, os.O_RDONLY)
        try:
            data = os.read(fd, 1024)
        finally:
            os.close(fd)
        s = data.decode(errors="ignore").strip()
        if not s:
            return None

        # Support both comma- and space-separated formats
        parts = [p for p in s.replace(",", " ").split() if p]
        values = []
        for p in parts:
            try:
                values.append(int(p))
            except ValueError:
                # Unexpected content, treat as unreadable
                return None
        return tuple(values)
    except Exception:
        return None


def led_matches(color_4tuple, current):
    """
    Compare a configured (A,R,G,B) color with the current LED.

    The driver may expose either 4 numbers (A,R,G,B) or just 3 (R,G,B),
    so we match accordingly.
    """
    if current is None:
        return False

    # Exact match (brightness + RGB)
    if len(current) == 4:
        return tuple(current) == tuple(color_4tuple)

    # Match RGB only if driver exposes just 3 values
    if len(current) == 3:
        return tuple(current) == tuple(color_4tuple[1:])

    return False


# Load default LED from config (in correct device order)
DEFAULT_LED = read_led_color_from_conf(
    "LED_COLOR_ONBOARDING",
    default=(255, 0, 0, 168),
)

# Static green LED for a running pipeline
GREEN_LED = (255, 0, 15, 0)  # (brightness, R, G, B)

def handle_led_on_exit():
    # On exit, if the LED is set to our green we set it back to default,
    # otherwise we let it as is.
    current_led = read_current_led()
    if led_matches(GREEN_LED, current_led):
        turn_on_led(*DEFAULT_LED)

def on_start():
    print("on_start")
    subprocess.call(['bash', str(current_dir / './on_start.sh')])

    # If the LED is default we change it to our green,
    # if it's not we let it be as is.
    current_led = read_current_led()
    if led_matches(DEFAULT_LED, current_led):
        turn_on_led(*GREEN_LED)

    env = shell_source(str(current_dir / './env_set.sh'))
    add_udo_prepare_directory(env)
    return env


def add_udo_prepare_directory(env):
    """Create an empty directory for NNArchive UDO online-prepare libraries and add it to LD_LIBRARY_PATH.

    SNPE loads the AArch64 "<DSP library>_AltPrep.so" of a UDO package through the
    dynamic loader, which reads LD_LIBRARY_PATH only when the firmware starts. The
    firmware stages those libraries here (UdoRuntime.hpp); on_destroy.sh removes it.
    """
    import shutil

    prepare_dir = current_dir.parent / '.depthai-udo-prepare'
    try:
        # Libraries of an earlier session that ended without on_destroy.
        shutil.rmtree(prepare_dir, ignore_errors=True)
        prepare_dir.mkdir(mode=0o700)
    except OSError as e:
        # The firmware reports the missing directory if an archive needs it.
        print(f"Cannot create {prepare_dir}: {e}")
        return
    library_path = env.get('LD_LIBRARY_PATH', '')
    env['LD_LIBRARY_PATH'] = f"{library_path}:{prepare_dir}" if library_path else str(prepare_dir)


def on_stop():
    handle_led_on_exit()

def on_create():
    print("on_create")
    pass

def on_destroy():
    handle_led_on_exit()
    subprocess.call(['bash', str(current_dir / './on_destroy.sh')])
    pass
