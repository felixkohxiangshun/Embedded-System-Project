import os
import argparse
import select
import shutil
import subprocess
import time
import uuid

def exchange(source_ws, destination_ws, qos, tcp_host="127.0.0.1"):
    pub = shutil.which("mosquitto_pub")
    sub = shutil.which("mosquitto_sub")
    if not pub or not sub:
        raise RuntimeError("Install Mosquitto and put its clients on PATH first")
    topic = "csc11/dev/broker-test/" + uuid.uuid4().hex
    payload = bytes.fromhex("8d230100ff")

    def transport(websocket):
        return ["-h", "127.0.0.1" if websocket else tcp_host, "-p", "9001" if websocket else "1883"] + (
            ["--ws"] if websocket else []
        )

    receiver = subprocess.Popen(
        ["/usr/bin/script", "-q", "/dev/null", sub, *transport(destination_ws), "-t", topic, "-q", str(qos),
         "-C", "1", "-W", "8", "-F", "%x", "-d"],
        stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        bufsize=0,
    )
    master = receiver.stdout.fileno()
    output = b""
    try:

        deadline = time.monotonic() + 8
        while b"received SUBACK" not in output:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([master], [], [], remaining)[0]:
                raise RuntimeError("Timed out waiting for SUBACK: " + output.decode(errors="replace"))
            chunk = os.read(master, 4096)
            if not chunk:
                raise RuntimeError("Subscriber exited: " + output.decode(errors="replace"))
            output += chunk

        subprocess.run(
            [pub, *transport(source_ws), "-t", topic, "-q", str(qos), "-s"],
            input=payload, check=True, timeout=8,
        )
        tail, _ = receiver.communicate(timeout=10)
        output += tail
        if receiver.returncode != 0 or payload.hex().encode() not in output.splitlines():
            raise RuntimeError("Binary mismatch: " + output.decode(errors="replace"))
        source = "WebSocket" if source_ws else "TCP"
        destination = "WebSocket" if destination_ws else "TCP"
        print(f"PASS {source} -> {destination}, QoS {qos}: {payload.hex()}")
    finally:
        if receiver.poll() is None:
            receiver.kill()
            receiver.wait()
        receiver.stdout.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tcp-host", default="127.0.0.1", help="Mac LAN IPv4 to test its Pico-facing listener")
    args = parser.parse_args()
    exchange(False, True, 0, args.tcp_host)
    exchange(True, False, 1, args.tcp_host)
    print("Broker smoke test PASS; no Pico or physical stopping behavior tested.")
