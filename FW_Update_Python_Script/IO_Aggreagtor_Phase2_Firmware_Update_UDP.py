import argparse  # For command-line argument parsing
import socket  # For UDP communication
import struct  # For packing/unpacking binary data
import tkinter as tk  # For GUI creation
from tkinter import filedialog, messagebox, scrolledtext, ttk
import threading  # To run update in a separate thread
import sys  # To access command-line arguments
import time  # For timestamping and delays

# ==============================
# Bootloader Constants
# ==============================
PACKET_SIZE = 64   # Fixed packet size for UDP communication
SOH = 0x01         # Start of Header
EOT = 0x04         # End of Transmission
DLE = 0x10         # Data Link Escape (used to escape control characters)

# Bootloader command identifiers
CMD_READ_VERSION = 0x01
CMD_ERASE_FLASH = 0x02
CMD_PROGRAM_FLASH = 0x03
CMD_READ_CRC = 0x04
CMD_START_APP = 0x05

# Global variables for GUI and logging
DEBUG = False
gui_console = None
root_window = None
start_btn = None
progress_bar = None
progress_label = None


# ==============================
# Logging Utilities
# ==============================
def log_message(message):
    """Logs a message with timestamp to console and GUI (if enabled)."""
    timestamp = time.strftime("[%H:%M:%S] ")
    msg = timestamp + message
    print(msg)
    if gui_console:
        gui_console.config(state=tk.NORMAL)
        gui_console.insert(tk.END, msg + "\n")
        gui_console.see(tk.END)
        gui_console.config(state=tk.DISABLED)


def debug_print(message):
    """Prints debug messages only if DEBUG flag is enabled."""
    if DEBUG:
        log_message(f"[DEBUG] {message}")


# ==============================
# CRC Calculation
# ==============================
def bootloader_CalculateCrc(data):
    """Calculates CRC-16 for bootloader communication."""
    crc_table = [
        0x0000, 0x1021, 0x2042, 0x3063,
        0x4084, 0x50a5, 0x60c6, 0x70e7,
        0x8108, 0x9129, 0xa14a, 0xb16b,
        0xc18c, 0xd1ad, 0xe1ce, 0xf1ef
    ]
    crc = 0
    len_data = len(data)
    while len_data != 0:
        i = ((crc >> 12) ^ (data[0] >> 4))
        crc = crc_table[i & 0x0F] ^ (crc << 4)
        i = ((crc >> 12) ^ (data[0] >> 0))
        crc = crc_table[i & 0x0F] ^ (crc << 4)
        data = data[1:]
        len_data -= 1
    return crc & 0xFFFF


# ==============================
# Frame Encoding & Decoding
# ==============================
def encode_frame(command, payload=[]):
    """
    Encodes a bootloader command into a frame with CRC and escape handling.
    - Escapes SOH, EOT, and DLE control characters.
    """
    data = [command] + payload
    crc_bootloader = bootloader_CalculateCrc(data)

    frame = [SOH]
    # Escape reserved bytes in data
    for byte in data:
        if byte in (SOH, EOT, DLE):
            frame.append(DLE)
        frame.append(byte)
    # Append CRC (little-endian) with escaping
    for byte in struct.pack("<H", crc_bootloader):
        if byte in (SOH, EOT, DLE):
            frame.append(DLE)
        frame.append(byte)
    frame.append(EOT)
    return frame


def read_response(sock, timeout=10):
    """
    Reads and decodes a response frame from the device.
    - Removes escape sequences.
    - Returns a list of bytes or None on timeout.
    """
    sock.settimeout(timeout)
    try:
        response, _ = sock.recvfrom(1024)
        decoded_response = []
        i = 0
        while i < len(response):
            if response[i] == DLE:  # Skip escape marker
                if i + 1 < len(response):
                    decoded_response.append(response[i + 1])
                    i += 2
                else:
                    break
            else:
                decoded_response.append(response[i])
                i += 1
        return decoded_response
    except socket.timeout:
        return None


def send_command(sock, ip, port, command, payload=[]):
    """
    Sends a command frame and waits for a response.
    - Retries up to 6 times if no response.
    - Raises ConnectionError on failure.
    """
    frame = encode_frame(command, payload)
    # Pad frame to fixed size
    frame += [0xFF] * (PACKET_SIZE - len(frame))
    debug_print(f"Encoded Frame: {[hex(b) for b in frame]}")

    for retry in range(6):
        try:
            sock.sendto(bytes(frame), (ip, port))
            resp = read_response(sock)
            if resp:
                return resp
            log_message(f"Retry {retry + 1}... No response")
        except socket.timeout:
            log_message(f"Retry {retry + 1}... socket timeout")

    log_message("ERROR: No response after 6 retries.")
    raise ConnectionError("No response from device")


# ==============================
# Bootloader Command Wrappers
# ==============================
def read_bootloader_version(sock, ip, port):
    """Reads and logs the bootloader version from device."""
    log_message("Reading bootloader version...")
    response = send_command(sock, ip, port, CMD_READ_VERSION)
    if response and response[0] == SOH and response[1] == CMD_READ_VERSION:
        log_message(f"Bootloader Version: {response[2]}.{response[3]}")
    else:
        log_message("Error: Invalid or no response.")
        return False
    return True


def erase_flash(sock, ip, port):
    """Sends erase flash command and checks response."""
    log_message("Erasing flash...")
    response = send_command(sock, ip, port, CMD_ERASE_FLASH)
    if response and response[0] == SOH and response[1] == CMD_ERASE_FLASH:
        log_message("Flash erase complete.")
    else:
        log_message("Error: Erase failed.")
        return False
    return True


def calculate_hex_record_checksum(data):
    """Calculates Intel HEX record checksum (two's complement)."""
    data_sum = sum(data)
    lsb = data_sum & 0xFF
    return (~lsb + 1) & 0xFF


def program_flash(sock, ip, port, data):
    """Programs one record of flash and validates response."""
    response = send_command(sock, ip, port, CMD_PROGRAM_FLASH, data)
    if not (response and response[0] == SOH and response[1] == CMD_PROGRAM_FLASH):
        log_message("Error: Program flash failed.")
        return False
    return True


def process_hex_file(hex_file_path, sock, ip, port):
    """
    Parses and programs the Intel HEX file line by line.
    - Validates checksums.
    - Updates GUI progress bar and ETA.
    """
    try:
        with open(hex_file_path, 'r') as hex_file:
            lines = [line.strip() for line in hex_file if line.strip().startswith(":")]
            total_lines = len(lines)
            start_time = time.time()

            for idx, line in enumerate(lines, start=1):
                # Convert hex record string to list of byte values
                record_data = [int(line[i:i + 2], 16) for i in range(1, len(line), 2)]
                # Verify checksum
                checksum_calculated = calculate_hex_record_checksum(record_data[:-1])
                if checksum_calculated != record_data[-1]:
                    log_message(f"Checksum mismatch: {line}")
                    continue
                # Program record to device
                if not program_flash(sock, ip, port, record_data):
                    return False

                # Update progress (about every 2%)
                if root_window and (idx % max(1, total_lines // 50) == 0 or idx == total_lines):
                    percent = int((idx / total_lines) * 100)
                    elapsed = time.time() - start_time
                    est_total = (elapsed / idx) * total_lines
                    remaining = max(0, est_total - elapsed)

                    root_window.after(0, lambda p=percent: progress_bar.config(value=p))
                    root_window.after(
                        0,
                        lambda r=remaining: progress_label.config(
                            text=f"{int(r)}s remaining..."
                        )
                    )

            log_message("Programming complete.")
            return True
    except FileNotFoundError:
        log_message(f"File not found: {hex_file_path}")
        return False


def jump_to_application(sock, ip, port):
    """Sends command to start user application on device."""
    log_message("Jumping to application...")
    try:
        response = send_command(sock, ip, port, CMD_START_APP)
        if response and response[0] == SOH and response[1] == CMD_START_APP:
            log_message("Jump accepted.")
        else:
            log_message("Jump command sent, no confirmation received.")
    except socket.timeout:
        log_message("Jump successful (no response).")


# ==============================
# Update Workflow
# ==============================
def run_update(hex_path, ip, port, debug_flag, max_retries=3):
    """
    Full firmware update workflow:
    1. Read bootloader version.
    2. Erase flash.
    3. Program HEX file.
    4. Jump to application.
    Retries on failure.
    """
    global DEBUG
    DEBUG = debug_flag

    attempt = 0
    success = False

    while attempt < max_retries and not success:
        attempt += 1
        log_message(f"=== Update Attempt {attempt}/{max_retries} ===")

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(5)
        try:
            if not read_bootloader_version(sock, ip, port):
                raise ConnectionError("Failed to read version")

            if not erase_flash(sock, ip, port):
                raise ConnectionError("Failed to erase flash")

            if not process_hex_file(hex_path, sock, ip, port):
                raise ConnectionError("Failed to program flash")

            jump_to_application(sock, ip, port)
            success = True

        except (socket.error, ConnectionError) as e:
            log_message(f"Connection lost or error: {e}. Retrying from start...")

        finally:
            sock.close()

    # Show result in GUI (if running)
    if root_window:
        root_window.after(0, lambda: messagebox.showinfo(
            "Update Result",
            "Firmware Update Successful!" if success else "Firmware Update Failed!"
        ))
        root_window.after(0, lambda: start_btn.config(state=tk.NORMAL))
        if gui_console:
            root_window.after(0, lambda: log_message(
                "=== Firmware Update Completed Successfully ===" if success else "=== Firmware Update Failed ==="
            ))


# ==============================
# GUI Implementation
# ==============================
def run_gui():
    """Creates and runs the Tkinter-based firmware update GUI."""
    global gui_console, root_window, start_btn, progress_bar, progress_label

    root = tk.Tk()
    root.title("IO Aggregator Firmware Update")
    root.geometry("800x650")
    root.minsize(800, 650)
    root.configure(bg="#f0f2f5")
    root_window = root

    # Header with title
    header = tk.Canvas(root, height=70, bg="#4facfe", highlightthickness=0)
    header.pack(fill="x")
    header.create_text(400, 35, text="IO Aggregator Firmware Update",
                       font=("Segoe UI", 18, "bold"), fill="white")

    content = tk.Frame(root, bg="#f0f2f5")
    content.pack(fill="both", expand=True, padx=25, pady=15)

    # Input fields
    hex_path_var = tk.StringVar()
    ip_var = tk.StringVar(value="192.168.1.11")
    port_var = tk.IntVar(value=6234)
    debug_var = tk.BooleanVar(value=False)

    # Browse file dialog
    def browse_file():
        file_path = filedialog.askopenfilename(
            title="Select HEX File",
            filetypes=[("HEX files", "*.hex"), ("All files", "*.*")]
        )
        if file_path:
            hex_path_var.set(file_path)

    # Start update button handler
    def start_update():
        start_btn.config(state=tk.DISABLED)
        gui_console.config(state=tk.NORMAL)
        gui_console.delete(1.0, tk.END)
        gui_console.insert(tk.END, "[INFO] PROCESSING... Please wait until the update finishes.\n")
        gui_console.config(state=tk.DISABLED)
        progress_bar.config(value=0)
        progress_label.config(text="")
        threading.Thread(
            target=lambda: run_update(
                hex_path_var.get(),
                ip_var.get(),
                port_var.get(),
                debug_var.get()
            ),
            daemon=True
        ).start()

    # Form layout
    tk.Label(content, text="HEX File:", bg="#f0f2f5", font=("Segoe UI", 10)).grid(
        row=0, column=0, sticky="w", pady=5
    )
    tk.Entry(content, textvariable=hex_path_var, width=55).grid(
        row=0, column=1, pady=5, padx=5, sticky="we"
    )
    tk.Button(content, text="Browse", command=browse_file, width=10).grid(
        row=0, column=2, padx=5
    )

    tk.Label(content, text="IP Address:", bg="#f0f2f5", font=("Segoe UI", 10)).grid(
        row=1, column=0, sticky="w", pady=5
    )
    tk.Entry(content, textvariable=ip_var, width=25).grid(
        row=1, column=1, sticky="w", pady=5
    )

    tk.Label(content, text="Port:", bg="#f0f2f5", font=("Segoe UI", 10)).grid(
        row=2, column=0, sticky="w", pady=5
    )
    tk.Entry(content, textvariable=port_var, width=10).grid(
        row=2, column=1, sticky="w", pady=5
    )

    tk.Checkbutton(content, text="Enable Debug", variable=debug_var, bg="#f0f2f5").grid(
        row=3, column=1, sticky="w", pady=5
    )

    # Progress bar
    progress_bar = ttk.Progressbar(content, length=500, mode="determinate")
    progress_bar.grid(row=4, column=0, columnspan=2, pady=20, sticky="w")
    style = ttk.Style()
    style.theme_use("default")
    style.configure("TProgressbar", thickness=20, troughcolor="#e0e0e0", background="green")

    progress_label = tk.Label(content, text="", bg="#f0f2f5", font=("Segoe UI", 10))
    progress_label.grid(row=4, column=2, sticky="w", padx=10)

    # Start update button
    start_btn = tk.Button(content, text="Start Update", command=start_update,
                          bg="green", fg="white", font=("Segoe UI", 11, "bold"),
                          width=15, height=1)
    start_btn.grid(row=5, column=1, pady=15)

    # Console output area
    gui_console = scrolledtext.ScrolledText(
        content, wrap=tk.WORD, height=18, width=95,
        state=tk.DISABLED, bg="black", fg="lime", font=("Consolas", 10)
    )
    gui_console.grid(row=6, column=0, columnspan=3, pady=15, sticky="nsew")

    # Allow console to expand on resize
    content.grid_rowconfigure(6, weight=1)
    content.grid_columnconfigure(1, weight=1)

    root.mainloop()


# ==============================
# Entry Point
# ==============================
if __name__ == "__main__":
    # CLI mode: run update directly
    if len(sys.argv) > 1:
        parser = argparse.ArgumentParser(description="UDP Bootloader Tool")
        parser.add_argument("HEXFILE", help="Path to HEX file")
        parser.add_argument("--ip", default="192.168.1.11", help="Device IP")
        parser.add_argument("--port", type=int, default=6234, help="UDP Port")
        parser.add_argument("--debug", action="store_true", help="Enable debug mode")
        args = parser.parse_args()
        run_update(args.HEXFILE, args.ip, args.port, args.debug)
    else:
        # GUI mode
        run_gui()
