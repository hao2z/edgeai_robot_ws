"""Create a local Xauthority cookie compatible with this VcXsrv release."""
import secrets
import struct
from pathlib import Path

# Older Windows Xauth readers can treat control bytes as text delimiters.
# An empty wildcard address also avoids a binary 0x1a in the WSL gateway IP.
cookie = bytes(secrets.choice(range(33, 127)) for _ in range(32))
fields = [b'', b'1', b'MIT-MAGIC-COOKIE-1', cookie]
record = struct.pack('!H', 65535)
record += b''.join(struct.pack('!H', len(field)) + field for field in fields)
(Path(__file__).resolve().parents[2] / 'runtime/vcxsrv.xauth').write_bytes(record)
