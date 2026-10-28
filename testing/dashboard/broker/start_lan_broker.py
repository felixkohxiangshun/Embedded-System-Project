import ipaddress
from pathlib import Path
import shutil
import subprocess
import tempfile

def main():
    broker = shutil.which("mosquitto") or "/opt/homebrew/sbin/mosquitto"
    if not Path(broker).is_file():
        raise SystemExit("Mosquitto not found; install with brew install mosquitto")
    result = subprocess.run(
        ["/usr/sbin/ipconfig", "getifaddr", "en0"],
        capture_output=True, text=True,
    )
    try:
        address = ipaddress.IPv4Address(result.stdout.strip())
    except ipaddress.AddressValueError:
        raise SystemExit("No IPv4 address on en0. Connect the Mac to your test WiFi first.")
    if address.is_loopback or address.is_unspecified or address.is_link_local:
        raise SystemExit("WiFi needs a usable LAN IPv4 address first.")

    with tempfile.TemporaryDirectory(prefix="csc11-broker-") as folder:
        root = Path(folder)
        acl = root / "development.acl"
        acl.write_text("topic readwrite csc11/dev/#\n")
        acl.chmod(0o600)
        config = root / "mosquitto.conf"
        config.write_text(
            "allow_anonymous true\npersistence false\n"
            f"acl_file {acl}\n"
            "log_dest stdout\nlog_type error\nlog_type warning\n"
            "log_type notice\nconnection_messages true\n"
            "listener 1883 127.0.0.1\nprotocol mqtt\n"
            f"listener 1883 {address}\nprotocol mqtt\n"
            "listener 9001 127.0.0.1\nprotocol websockets\n"
        )
        config.chmod(0o600)
        print(f"Pico broker address: {address}:1883", flush=True)
        print("Webpage transport: ws://127.0.0.1:9001/mqtt", flush=True)
        print("Development topics only: csc11/dev/#; Ctrl+C stops the broker.", flush=True)
        process = subprocess.Popen([broker, "-c", str(config), "-v"])
        try:
            return process.wait()
        except KeyboardInterrupt:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            return 0

if __name__ == "__main__":
    raise SystemExit(main())
